from __future__ import annotations

import argparse
import json
import math
import random
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence

from pymavlink import mavutil


DRONE_PORTS = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}

DEFAULT_BUILDINGS_FILE = Path("ucy_buildings.txt")
DEFAULT_CORNERS_FILE = Path("ucy_building_corners.txt")
DEFAULT_EVENT_LOG = Path("mission_events.jsonl")
DEFAULT_ALTITUDE_M = 10.0
DEFAULT_SCAN_DELAY_S = 2.0
WAYPOINT_REACH_THRESHOLD_M = 3.0
POSITION_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class Position:
    lat: float
    lon: float
    alt: float


@dataclass(frozen=True)
class BuildingPlan:
    name: str
    center: Position
    probability: float
    half_north_m: float
    half_east_m: float


@dataclass(frozen=True)
class CornerPlan:
    building_name: str
    corner_name: str
    position: Position
    probability: float


@dataclass(frozen=True)
class DetectionAttempt:
    building_name: str
    corner_name: str
    probability: float
    random_value: float
    detected: bool


class MissionEventLogger:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.write_text("", encoding="utf-8")

    def log(self, event: str, message: str, **extra: object) -> None:
        payload = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "event": event,
            "message": message,
        }
        payload.update(extra)
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload) + "\n")


class DroneConnection:
    def __init__(self, name: str, endpoint: str) -> None:
        self.name = name
        self.endpoint = endpoint
        self.link: Optional[mavutil.mavfile] = None

    def connect(self) -> None:
        if self.link is not None:
            return
        print(f"{self.name}: connecting on {self.endpoint} ...")
        link = mavutil.mavlink_connection(self.endpoint, source_system=250)
        heartbeat = link.wait_heartbeat(timeout=20)
        link.target_system = heartbeat.get_srcSystem()
        link.target_component = heartbeat.get_srcComponent()
        self.link = link
        print(
            f"{self.name}: connected with sysid={link.target_system} "
            f"compid={link.target_component}"
        )

    def require_link(self) -> mavutil.mavfile:
        if self.link is None:
            raise RuntimeError(f"{self.name}: drone is not connected")
        return self.link

    def wait_for_position(self, timeout_s: float = POSITION_TIMEOUT_S) -> Position:
        link = self.require_link()
        message = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout_s)
        if not message:
            raise RuntimeError(f"{self.name}: timed out waiting for GLOBAL_POSITION_INT")
        return Position(
            lat=message.lat / 1e7,
            lon=message.lon / 1e7,
            alt=message.relative_alt / 1000.0,
        )

    def set_guided(self) -> None:
        link = self.require_link()
        mapping = link.mode_mapping()
        if not mapping or "GUIDED" not in mapping:
            raise RuntimeError(f"{self.name}: GUIDED mode is not available")
        mode_id = mapping["GUIDED"]
        link.mav.set_mode_send(
            link.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print(f"{self.name}: switching to GUIDED")
        time.sleep(1)

    def arm(self) -> None:
        link = self.require_link()
        print(f"{self.name}: arming")
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        time.sleep(2)

    def takeoff(self, altitude_m: float) -> None:
        link = self.require_link()
        print(f"{self.name}: taking off to {altitude_m:.1f} m")
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude_m,
        )
        self.wait_until_altitude(altitude_m * 0.9)

    def wait_until_altitude(self, minimum_altitude_m: float, timeout_s: float = 45.0) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            position = self.wait_for_position()
            print(f"{self.name}: altitude={position.alt:.1f} m")
            if position.alt >= minimum_altitude_m:
                return
            time.sleep(1)
        raise RuntimeError(f"{self.name}: takeoff timed out")

    def goto(self, position: Position) -> None:
        link = self.require_link()
        print(
            f"{self.name}: going to lat={position.lat:.6f}, "
            f"lon={position.lon:.6f}, alt={position.alt:.1f} m"
        )
        link.mav.set_position_target_global_int_send(
            0,
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b110111111000),
            int(position.lat * 1e7),
            int(position.lon * 1e7),
            position.alt,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def wait_until_close(self, target: Position, threshold_m: float = WAYPOINT_REACH_THRESHOLD_M) -> Position:
        while True:
            current = self.wait_for_position()
            separation = distance_m(current, target)
            print(
                f"{self.name}: now at lat={current.lat:.6f}, lon={current.lon:.6f}, "
                f"alt={current.alt:.1f} m, dist={separation:.1f} m"
            )
            if separation <= threshold_m:
                return current
            time.sleep(1)


def distance_m(a: Position, b: Position) -> float:
    dx = (b.lon - a.lon) * 111320.0 * math.cos(math.radians((a.lat + b.lat) / 2.0))
    dy = (b.lat - a.lat) * 110540.0
    dz = b.alt - a.alt
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def offset_position(center: Position, north_m: float, east_m: float) -> Position:
    lat_delta = north_m / 110540.0
    lon_delta = east_m / (111320.0 * math.cos(math.radians(center.lat)))
    return Position(
        lat=center.lat + lat_delta,
        lon=center.lon + lon_delta,
        alt=center.alt,
    )


def load_buildings(buildings_file: Path) -> List[BuildingPlan]:
    buildings: List[BuildingPlan] = []
    for line_number, raw_line in enumerate(buildings_file.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        parts = [part.strip() for part in line.split(",")]
        if len(parts) != 5:
            raise ValueError(
                f"{buildings_file}:{line_number} must contain name,lat,lon,altitude,probability"
            )

        name = parts[0]
        lat = float(parts[1])
        lon = float(parts[2])
        altitude_m = float(parts[3])
        probability = float(parts[4])

        # Generated rectangular coverage dimensions around each center point.
        buildings.append(
            BuildingPlan(
                name=name,
                center=Position(lat=lat, lon=lon, alt=altitude_m),
                probability=probability,
                half_north_m=18.0,
                half_east_m=12.0,
            )
        )

    if not buildings:
        raise ValueError(f"No usable buildings were found in {buildings_file}")

    return buildings


def load_exact_corners(corners_file: Path, buildings: Sequence[BuildingPlan]) -> dict[str, List[CornerPlan]]:
    if not corners_file.exists():
        return {}

    building_map = {building.name: building for building in buildings}
    exact: dict[str, List[CornerPlan]] = {}
    for line_number, raw_line in enumerate(corners_file.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        parts = [part.strip() for part in line.split(",")]
        if len(parts) != 9:
            raise ValueError(
                f"{corners_file}:{line_number} must contain name and 4 lat/lon corner pairs"
            )

        name = parts[0]
        if name not in building_map:
            continue

        building = building_map[name]
        values = [float(value) for value in parts[1:]]
        corners: List[CornerPlan] = []
        corner_names = ["P1", "P2", "P3", "P4"]
        for index, corner_name in enumerate(corner_names):
            lat = values[index * 2]
            lon = values[index * 2 + 1]
            corners.append(
                CornerPlan(
                    building_name=name,
                    corner_name=corner_name,
                    position=Position(lat=lat, lon=lon, alt=building.center.alt),
                    probability=building.probability,
                )
            )
        exact[name] = corners

    return exact


def build_generated_corners(building: BuildingPlan) -> List[CornerPlan]:
    offsets = [
        ("NW", building.half_north_m, -building.half_east_m),
        ("NE", building.half_north_m, building.half_east_m),
        ("SE", -building.half_north_m, building.half_east_m),
        ("SW", -building.half_north_m, -building.half_east_m),
    ]
    return [
        CornerPlan(
            building_name=building.name,
            corner_name=corner_name,
            position=offset_position(building.center, north_m, east_m),
            probability=building.probability,
        )
        for corner_name, north_m, east_m in offsets
    ]


def prepare_for_mission(drone: DroneConnection, altitude_m: float, event_logger: MissionEventLogger) -> None:
    drone.connect()
    home = drone.wait_for_position()
    print(
        f"{drone.name}: home position lat={home.lat:.6f}, "
        f"lon={home.lon:.6f}, alt={home.alt:.1f} m"
    )
    event_logger.log("home_position", f"{drone.name} home acquired", drone=drone.name, lat=home.lat, lon=home.lon)
    drone.set_guided()
    event_logger.log("mode_change", f"{drone.name} switched to GUIDED", drone=drone.name)
    drone.arm()
    event_logger.log("arm", f"{drone.name} armed", drone=drone.name)
    drone.takeoff(altitude_m)
    event_logger.log("takeoff_complete", f"{drone.name} reached takeoff altitude", drone=drone.name, altitude=altitude_m)


def send_observer(observer: DroneConnection, target: CornerPlan, event_logger: MissionEventLogger) -> Position:
    event_logger.log(
        "response_launch",
        f"{observer.name} launching toward {target.building_name} {target.corner_name}",
        drone=observer.name,
        building=target.building_name,
        corner=target.corner_name,
        lat=target.position.lat,
        lon=target.position.lon,
    )
    prepare_for_mission(observer, target.position.alt, event_logger)
    observer.goto(target.position)
    final = observer.wait_until_close(target.position)
    event_logger.log(
        "response_arrival",
        f"{observer.name} reached {target.building_name} {target.corner_name}",
        drone=observer.name,
        building=target.building_name,
        corner=target.corner_name,
        lat=final.lat,
        lon=final.lon,
    )
    return final


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Drone 1 scans generated building rectangles; drone 2 launches immediately to first detected point."
    )
    parser.add_argument("--buildings-file", default=str(DEFAULT_BUILDINGS_FILE))
    parser.add_argument("--corners-file", default=str(DEFAULT_CORNERS_FILE))
    parser.add_argument("--event-log", default=str(DEFAULT_EVENT_LOG))
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--scan-delay", type=float, default=DEFAULT_SCAN_DELAY_S)
    parser.add_argument("--altitude", type=float, default=DEFAULT_ALTITUDE_M)
    args = parser.parse_args()

    event_logger = MissionEventLogger(Path(args.event_log))
    buildings = load_buildings(Path(args.buildings_file))
    exact_corners_by_building = load_exact_corners(Path(args.corners_file), buildings)

    print("Generated building rectangles:")
    for building in buildings:
        if building.name in exact_corners_by_building:
            print(f"- {building.name}: using exact corner coordinates from file, probability={building.probability:.0%}")
        else:
            print(
                f"- {building.name}: center=({building.center.lat:.6f}, {building.center.lon:.6f}), "
                f"rectangle={building.half_north_m * 2:.0f}m x {building.half_east_m * 2:.0f}m, "
                f"probability={building.probability:.0%}"
            )

    scout = DroneConnection("drone1", DRONE_PORTS["drone1"])
    observer = DroneConnection("drone2", DRONE_PORTS["drone2"])

    rng = random.Random(args.seed)
    prepare_for_mission(scout, args.altitude, event_logger)

    attempts: List[DetectionAttempt] = []
    observer_launched = False
    observer_target: Optional[CornerPlan] = None

    for building in buildings:
        corners = exact_corners_by_building.get(building.name, build_generated_corners(building))
        event_logger.log(
            "building_start",
            f"{scout.name} starting rectangle scan for {building.name}",
            drone=scout.name,
            building=building.name,
        )

        for corner in corners:
            print(f"\n{scout.name}: flying to {building.name} {corner.corner_name}")
            event_logger.log(
                "scan_move",
                f"{scout.name} flying to {building.name} {corner.corner_name}",
                drone=scout.name,
                building=building.name,
                corner=corner.corner_name,
                probability=corner.probability,
                lat=corner.position.lat,
                lon=corner.position.lon,
            )
            scout.goto(corner.position)
            scout.wait_until_close(corner.position)

            print(f"{scout.name}: scanning at {building.name} {corner.corner_name} for {args.scan_delay:.1f} s")
            event_logger.log(
                "scan_start",
                f"{scout.name} scanning {building.name} {corner.corner_name}",
                drone=scout.name,
                building=building.name,
                corner=corner.corner_name,
                duration=args.scan_delay,
            )
            time.sleep(args.scan_delay)

            random_value = rng.random()
            detected = random_value <= corner.probability
            attempts.append(
                DetectionAttempt(
                    building_name=building.name,
                    corner_name=corner.corner_name,
                    probability=corner.probability,
                    random_value=random_value,
                    detected=detected,
                )
            )

            print(
                f"{scout.name}: {building.name} {corner.corner_name} | "
                f"probability={corner.probability:.0%} | random={random_value:.3f} | "
                f"detected={'YES' if detected else 'NO'}"
            )
            event_logger.log(
                "scan_result",
                (
                    f"{scout.name} scan result at {building.name} {corner.corner_name}: "
                    f"probability={corner.probability:.0%}, random={random_value:.3f}, "
                    f"detected={'YES' if detected else 'NO'}"
                ),
                drone=scout.name,
                building=building.name,
                corner=corner.corner_name,
                probability=corner.probability,
                random_value=random_value,
                detected=detected,
            )

            if detected and not observer_launched:
                print(f"{scout.name}: person detected at {building.name} {corner.corner_name}")
                event_logger.log(
                    "person_detected",
                    f"{scout.name} detected a person at {building.name} {corner.corner_name}",
                    drone=scout.name,
                    building=building.name,
                    corner=corner.corner_name,
                    lat=corner.position.lat,
                    lon=corner.position.lon,
                )
                observer_target = corner
                observer_launched = True
                send_observer(observer, corner, event_logger)

        first_corner = corners[0]
        print(f"\n{scout.name}: returning to {building.name} {first_corner.corner_name}")
        event_logger.log(
            "building_return",
            f"{scout.name} returning to {building.name} {first_corner.corner_name} before next building",
            drone=scout.name,
            building=building.name,
            corner=first_corner.corner_name,
            lat=first_corner.position.lat,
            lon=first_corner.position.lon,
        )
        scout.goto(first_corner.position)
        scout.wait_until_close(first_corner.position)

    print("\nMission summary:")
    for attempt in attempts:
        print(
            f"- {attempt.building_name} {attempt.corner_name} | "
            f"probability={attempt.probability:.0%} | random={attempt.random_value:.3f} | "
            f"detected={'YES' if attempt.detected else 'NO'}"
        )

    if observer_target is None:
        print("- drone2: no launch, because no person was detected")
        event_logger.log("response_skipped", "drone2 did not launch because no person was detected", drone="drone2")
    else:
        print(f"- drone2: launched to {observer_target.building_name} {observer_target.corner_name}")
        event_logger.log(
            "mission_complete",
            f"Mission complete: drone2 launched to {observer_target.building_name} {observer_target.corner_name}",
            drone="drone2",
            building=observer_target.building_name,
            corner=observer_target.corner_name,
        )


if __name__ == "__main__":
    main()
