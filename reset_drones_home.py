from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil

DRONE_PORTS = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}

HOME_REACHED_DISTANCE_M = 3.0
HOME_REACHED_ALTITUDE_M = 1.0


@dataclass(frozen=True)
class Position:
    lat: float
    lon: float
    alt: float


def connect(name: str, endpoint: str) -> mavutil.mavfile:
    print(f"{name}: connecting on {endpoint} ...")
    link = mavutil.mavlink_connection(endpoint, source_system=250)
    heartbeat = link.wait_heartbeat(timeout=20)
    link.target_system = heartbeat.get_srcSystem()
    link.target_component = heartbeat.get_srcComponent()
    print(
        f"{name}: connected with sysid={link.target_system} "
        f"compid={link.target_component}"
    )
    return link


def wait_for_position(link: mavutil.mavfile, name: str, timeout_s: float = 5.0) -> Position:
    message = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout_s)
    if not message:
        raise RuntimeError(f"{name}: timed out waiting for GLOBAL_POSITION_INT")
    return Position(
        lat=message.lat / 1e7,
        lon=message.lon / 1e7,
        alt=message.relative_alt / 1000.0,
    )


def request_home_position(link: mavutil.mavfile, name: str, timeout_s: float = 10.0) -> Position:
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    message = link.recv_match(type="HOME_POSITION", blocking=True, timeout=timeout_s)
    if not message:
        raise RuntimeError(f"{name}: timed out waiting for HOME_POSITION")
    return Position(
        lat=message.latitude / 1e7,
        lon=message.longitude / 1e7,
        alt=0.0,
    )


def horizontal_distance_m(a: Position, b: Position) -> float:
    dx = (b.lon - a.lon) * 111320.0 * math.cos(math.radians((a.lat + b.lat) / 2.0))
    dy = (b.lat - a.lat) * 110540.0
    return math.sqrt(dx * dx + dy * dy)


def set_mode(link: mavutil.mavfile, name: str, mode_name: str) -> None:
    mode_mapping = link.mode_mapping()
    if not mode_mapping or mode_name not in mode_mapping:
        raise RuntimeError(f"{name}: mode {mode_name} is not available")
    mode_id = mode_mapping[mode_name]
    link.mav.set_mode_send(
        link.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    print(f"{name}: switching to {mode_name}")
    time.sleep(1)


def reset_to_home(name: str, endpoint: str) -> None:
    link = connect(name, endpoint)
    current = wait_for_position(link, name)
    home = request_home_position(link, name)

    start_distance = horizontal_distance_m(current, home)
    print(
        f"{name}: current lat={current.lat:.6f}, lon={current.lon:.6f}, alt={current.alt:.1f} m"
    )
    print(f"{name}: home lat={home.lat:.6f}, lon={home.lon:.6f}")

    if start_distance <= HOME_REACHED_DISTANCE_M and current.alt <= HOME_REACHED_ALTITUDE_M:
        print(f"{name}: already at home position")
        return

    set_mode(link, name, "RTL")

    deadline = time.time() + 120.0
    while time.time() < deadline:
        current = wait_for_position(link, name)
        distance = horizontal_distance_m(current, home)
        print(
            f"{name}: dist_to_home={distance:.1f} m, alt={current.alt:.1f} m"
        )
        if distance <= HOME_REACHED_DISTANCE_M and current.alt <= HOME_REACHED_ALTITUDE_M:
            print(f"{name}: reached home position")
            return
        time.sleep(1)

    raise RuntimeError(f"{name}: did not reach home position before timeout")


def main() -> None:
    for name, endpoint in DRONE_PORTS.items():
        reset_to_home(name, endpoint)


if __name__ == "__main__":
    main()
