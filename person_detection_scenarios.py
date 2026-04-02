from __future__ import annotations

import argparse
import json
import math
import random
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional, Sequence

from pymavlink import mavutil

DRONE_PORTS = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}

DEFAULT_PROBABILITIES = [0.25, 0.55, 0.85]
DEFAULT_ALTITUDE_M = 12.0
WAYPOINT_REACH_THRESHOLD_M = 3.0
POSITION_TIMEOUT_S = 5.0
DEFAULT_EVENT_LOG = Path("mission_events.jsonl")
DEFAULT_LOCATIONS_FILE = Path("ucy_buildings.txt")


@dataclass(frozen=True)
class DetectionAttempt:
    step: int
    location_name: str
    probability: float
    random_value: float
    detected: bool


@dataclass(frozen=True)
class LocationPlan:
    name: str
    lat: float
    lon: float
    altitude_m: float
    probability: float


@dataclass(frozen=True)
class Position:
    lat: float
    lon: float
    alt: float


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


def parse_probabilities(raw_values: Optional[Iterable[float]]) -> List[float]:
    if raw_values is None:
        return list(DEFAULT_PROBABILITIES)

    probabilities = [float(value) for value in raw_values]
    if len(probabilities) != 3:
        raise ValueError("Exactly three probabilities are required")

    for probability in probabilities:
        if not 0.0 <= probability <= 1.0:
            raise ValueError("Probabilities must be between 0.0 and 1.0")

    return probabilities


def add_detection_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument(
        "--probabilities",
        nargs=3,
        type=float,
        metavar=("P1", "P2", "P3"),
        help="Three detection probabilities, one for each location",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional random seed for repeatable simulation results",
    )
    parser.add_argument(
        "--scan-delay",
        type=float,
        default=2.0,
        help="Seconds to wait before evaluating detection at each location",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=DEFAULT_ALTITUDE_M,
        help="Mission altitude in meters",
    )
    parser.add_argument(
        "--event-log",
        default=str(DEFAULT_EVENT_LOG),
        help="Path to the JSONL event log consumed by the live map",
    )
    parser.add_argument(
        "--locations-file",
        default=str(DEFAULT_LOCATIONS_FILE),
        help="Text file with mission locations in the format name,lat,lon,altitude,probability",
    )
    return parser


def distance_m(a: Position, b: Position) -> float:
    dx = (b.lon - a.lon) * 111320.0 * math.cos(math.radians((a.lat + b.lat) / 2.0))
    dy = (b.lat - a.lat) * 110540.0
    dz = b.alt - a.alt
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def load_location_plan(locations_file: Path, fallback_probabilities: Sequence[float], default_altitude_m: float) -> List[tuple[LocationPlan, Position]]:
    if not locations_file.exists():
        raise FileNotFoundError(f"Locations file not found: {locations_file}")

    targets: List[tuple[LocationPlan, Position]] = []
    lines = locations_file.read_text(encoding="utf-8").splitlines()
    fallback_index = 0

    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        parts = [part.strip() for part in line.split(",")]
        if len(parts) not in (3, 4, 5):
            raise ValueError(
                f"{locations_file}:{line_number} must contain 3, 4, or 5 comma-separated values: "
                "name,lat,lon[,altitude][,probability]"
            )

        name = parts[0]
        lat = float(parts[1])
        lon = float(parts[2])
        altitude_m = float(parts[3]) if len(parts) >= 4 else default_altitude_m

        if len(parts) == 5:
            probability = float(parts[4])
        else:
            if fallback_index >= len(fallback_probabilities):
                raise ValueError(
                    f"{locations_file}:{line_number} does not include a probability and no fallback probability is available"
                )
            probability = fallback_probabilities[fallback_index]

        if not 0.0 <= probability <= 1.0:
            raise ValueError(f"{locations_file}:{line_number} probability must be between 0.0 and 1.0")

        location = LocationPlan(
            name=name,
            lat=lat,
            lon=lon,
            altitude_m=altitude_m,
            probability=probability,
        )
        targets.append(
            (
                location,
                Position(lat=lat, lon=lon, alt=altitude_m),
            )
        )
        fallback_index += 1

    if not targets:
        raise ValueError(f"No usable locations were found in {locations_file}")

    return targets


def prepare_for_mission(
    drone: DroneConnection,
    altitude_m: float,
    event_logger: MissionEventLogger,
) -> Position:
    drone.connect()
    home = drone.wait_for_position()
    print(
        f"{drone.name}: home position lat={home.lat:.6f}, "
        f"lon={home.lon:.6f}, alt={home.alt:.1f} m"
    )
    event_logger.log(
        "home_position",
        f"{drone.name} home position acquired",
        drone=drone.name,
        lat=home.lat,
        lon=home.lon,
        alt=home.alt,
    )
    drone.set_guided()
    event_logger.log("mode_change", f"{drone.name} switched to GUIDED", drone=drone.name)
    drone.arm()
    event_logger.log("arm", f"{drone.name} armed", drone=drone.name)
    drone.takeoff(altitude_m)
    event_logger.log(
        "takeoff_complete",
        f"{drone.name} reached takeoff altitude {altitude_m:.1f} m",
        drone=drone.name,
        altitude=altitude_m,
    )
    return home


def run_detection_mission(
    drone: DroneConnection,
    targets: Sequence[tuple[LocationPlan, Position]],
    *,
    seed: Optional[int],
    scan_delay_s: float,
    event_logger: MissionEventLogger,
) -> tuple[List[DetectionAttempt], List[tuple[LocationPlan, Position]]]:
    rng = random.Random(seed)
    attempts: List[DetectionAttempt] = []
    detections: List[tuple[LocationPlan, Position]] = []

    prepare_for_mission(drone, targets[0][1].alt, event_logger)

    for step, (location, target_position) in enumerate(targets, start=1):
        print(f"\n{drone.name}: flying to {location.name}")
        event_logger.log(
            "scan_move",
            f"{drone.name} flying to {location.name}",
            drone=drone.name,
            location=location.name,
            probability=location.probability,
            lat=target_position.lat,
            lon=target_position.lon,
            alt=target_position.alt,
        )
        drone.goto(target_position)
        drone.wait_until_close(target_position)
        event_logger.log(
            "scan_arrival",
            f"{drone.name} reached {location.name}",
            drone=drone.name,
            location=location.name,
        )

        if scan_delay_s > 0:
            print(f"{drone.name}: scanning at {location.name} for {scan_delay_s:.1f} s")
            event_logger.log(
                "scan_start",
                f"{drone.name} scanning {location.name}",
                drone=drone.name,
                location=location.name,
                duration=scan_delay_s,
            )
            time.sleep(scan_delay_s)

        random_value = rng.random()
        detected = random_value <= location.probability
        attempt = DetectionAttempt(
            step=step,
            location_name=location.name,
            probability=location.probability,
            random_value=random_value,
            detected=detected,
        )
        attempts.append(attempt)

        print(
            f"{drone.name}: {location.name} | probability={location.probability:.0%} | "
            f"random={random_value:.3f} | detected={'YES' if detected else 'NO'}"
        )
        event_logger.log(
            "scan_result",
            (
                f"{drone.name} scan result at {location.name}: "
                f"probability={location.probability:.0%}, random={random_value:.3f}, "
                f"detected={'YES' if detected else 'NO'}"
            ),
            drone=drone.name,
            location=location.name,
            probability=location.probability,
            random_value=random_value,
            detected=detected,
        )

        if detected:
            print(f"{drone.name}: person detected at {location.name}")
            event_logger.log(
                "person_detected",
                f"{drone.name} detected a person at {location.name}",
                drone=drone.name,
                location=location.name,
                lat=target_position.lat,
                lon=target_position.lon,
                alt=target_position.alt,
            )
            detections.append((location, target_position))

    print(f"{drone.name}: mission finished with no detection")
    event_logger.log("mission_no_detection", f"{drone.name} finished with no detection", drone=drone.name)
    return attempts, detections


def send_drone_directly_to_location(
    drone: DroneConnection,
    target_location: LocationPlan,
    target_position: Position,
    event_logger: MissionEventLogger,
) -> Position:
    print(
        f"\n{drone.name}: launching response to {target_location.name} "
        f"at lat={target_position.lat:.6f}, lon={target_position.lon:.6f}"
    )
    event_logger.log(
        "response_launch",
        f"{drone.name} launching toward {target_location.name}",
        drone=drone.name,
        location=target_location.name,
        lat=target_position.lat,
        lon=target_position.lon,
        alt=target_position.alt,
    )
    prepare_for_mission(drone, target_position.alt, event_logger)
    drone.goto(target_position)
    final_position = drone.wait_until_close(target_position)
    print(f"{drone.name}: reached {target_location.name}")
    event_logger.log(
        "response_arrival",
        f"{drone.name} reached {target_location.name}",
        drone=drone.name,
        location=target_location.name,
        lat=final_position.lat,
        lon=final_position.lon,
        alt=final_position.alt,
    )
    return final_position


def launch_observer_if_needed(
    observer: DroneConnection,
    detected_targets: Sequence[tuple[LocationPlan, Position]],
    event_logger: MissionEventLogger,
) -> Optional[tuple[LocationPlan, Position]]:
    if not detected_targets:
        return None

    first_target = detected_targets[0]
    observer_position = send_drone_directly_to_location(
        observer,
        first_target[0],
        first_target[1],
        event_logger,
    )
    return first_target[0], observer_position


def main() -> None:
    parser = add_detection_arguments(
        argparse.ArgumentParser(
            description="Drone 1 searches three waypoints; drone 2 responds to a confirmed detection."
        )
    )
    args = parser.parse_args()

    probabilities = parse_probabilities(args.probabilities)
    scout_seed = args.seed
    event_logger = MissionEventLogger(Path(args.event_log))

    scout = DroneConnection("drone1", DRONE_PORTS["drone1"])
    scout.connect()
    scout.wait_for_position()
    targets = load_location_plan(Path(args.locations_file), probabilities, args.altitude)

    print("Mission waypoints:")
    for location, target in targets:
        print(
            f"- {location.name}: lat={target.lat:.6f}, lon={target.lon:.6f}, "
            f"alt={target.alt:.1f} m, probability={location.probability:.0%}"
        )
    event_logger.log(
        "mission_start",
        "Mission started and waypoints were generated",
        waypoints=[
            {
                "name": location.name,
                "lat": target.lat,
                "lon": target.lon,
                "alt": target.alt,
                "probability": location.probability,
            }
            for location, target in targets
        ],
    )

    scout = DroneConnection("drone1", DRONE_PORTS["drone1"])
    observer = DroneConnection("drone2", DRONE_PORTS["drone2"])

    observer_result: Optional[tuple[LocationPlan, Position]] = None
    observer_launched = False
    scout_attempts: List[DetectionAttempt] = []
    detected_targets: List[tuple[LocationPlan, Position]] = []

    rng = random.Random(scout_seed)
    prepare_for_mission(scout, targets[0][1].alt, event_logger)

    for step, (location, target_position) in enumerate(targets, start=1):
        print(f"\n{scout.name}: flying to {location.name}")
        event_logger.log(
            "scan_move",
            f"{scout.name} flying to {location.name}",
            drone=scout.name,
            location=location.name,
            probability=location.probability,
            lat=target_position.lat,
            lon=target_position.lon,
            alt=target_position.alt,
        )
        scout.goto(target_position)
        scout.wait_until_close(target_position)
        event_logger.log(
            "scan_arrival",
            f"{scout.name} reached {location.name}",
            drone=scout.name,
            location=location.name,
        )

        if scan_delay_s := args.scan_delay:
            print(f"{scout.name}: scanning at {location.name} for {scan_delay_s:.1f} s")
            event_logger.log(
                "scan_start",
                f"{scout.name} scanning {location.name}",
                drone=scout.name,
                location=location.name,
                duration=scan_delay_s,
            )
            time.sleep(scan_delay_s)

        random_value = rng.random()
        detected = random_value <= location.probability
        attempt = DetectionAttempt(
            step=step,
            location_name=location.name,
            probability=location.probability,
            random_value=random_value,
            detected=detected,
        )
        scout_attempts.append(attempt)

        print(
            f"{scout.name}: {location.name} | probability={location.probability:.0%} | "
            f"random={random_value:.3f} | detected={'YES' if detected else 'NO'}"
        )
        event_logger.log(
            "scan_result",
            (
                f"{scout.name} scan result at {location.name}: "
                f"probability={location.probability:.0%}, random={random_value:.3f}, "
                f"detected={'YES' if detected else 'NO'}"
            ),
            drone=scout.name,
            location=location.name,
            probability=location.probability,
            random_value=random_value,
            detected=detected,
        )

        if detected:
            print(f"{scout.name}: person detected at {location.name}")
            event_logger.log(
                "person_detected",
                f"{scout.name} detected a person at {location.name}",
                drone=scout.name,
                location=location.name,
                lat=target_position.lat,
                lon=target_position.lon,
                alt=target_position.alt,
            )
            detected_targets.append((location, target_position))

            if not observer_launched:
                observer_result = launch_observer_if_needed(observer, detected_targets, event_logger)
                observer_launched = observer_result is not None

    if not detected_targets:
        print(f"{scout.name}: mission finished with no detection")
        event_logger.log("mission_no_detection", f"{scout.name} finished with no detection", drone=scout.name)
        print("\ndrone2: no launch, because drone1 did not find a person.")
        event_logger.log("response_skipped", "drone2 did not launch because no person was detected", drone="drone2")

    print("\nMission summary:")
    for attempt in scout_attempts:
        print(
            f"- drone1: {attempt.location_name} | probability={attempt.probability:.0%} | "
            f"random={attempt.random_value:.3f} | detected={'YES' if attempt.detected else 'NO'}"
        )

    if detected_targets and observer_result is not None:
        print(
            f"- drone2: dispatched to {observer_result[0].name} "
            f"(lat={observer_result[1].lat:.6f}, lon={observer_result[1].lon:.6f}, alt={observer_result[1].alt:.1f} m)"
        )
        event_logger.log(
            "mission_complete",
            f"Mission complete: drone2 dispatched to {observer_result[0].name}",
            drone="drone2",
            location=observer_result[0].name,
            lat=observer_result[1].lat,
            lon=observer_result[1].lon,
            alt=observer_result[1].alt,
        )
    else:
        print("- drone2: stayed at its start position")
        event_logger.log("mission_complete", "Mission complete: drone2 stayed at start position", drone="drone2")


if __name__ == "__main__":
    main()
