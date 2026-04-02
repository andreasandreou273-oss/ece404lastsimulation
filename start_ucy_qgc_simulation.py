from __future__ import annotations

import subprocess

DRONE_CONFIGS = (
    {
        "name": "drone1",
        "instance": 0,
        "sysid": 1,
        "python_port": 14550,
        "map_port": 14551,
        "qgc_port": 14555,
        "location": "35.144670,33.413750,50,0",
    },
    {
        "name": "drone2",
        "instance": 1,
        "sysid": 2,
        "python_port": 14560,
        "map_port": 14561,
        "qgc_port": 14556,
        "location": "35.144670,33.413900,50,0",
    },
    {
        "name": "drone3",
        "instance": 2,
        "sysid": 3,
        "python_port": 14570,
        "map_port": 14571,
        "qgc_port": 14557,
        "location": "35.144800,33.413750,50,0",
    },
)


def start_simulator(drone: dict[str, object]) -> None:
    sim_command = (
        "cd ~/ardupilot/ArduCopter && "
        "sim_vehicle.py "
        f"-v ArduCopter -I{drone['instance']} -w "
        f"--sysid {drone['sysid']} "
        f"--out udp:127.0.0.1:{drone['python_port']} "
        f"--out udp:127.0.0.1:{drone['map_port']} "
        f"--out udp:127.0.0.1:{drone['qgc_port']} "
        "--no-rebuild --console "
        f"--custom-location={drone['location']}"
    )

    subprocess.Popen(
        [
            "powershell",
            "-NoProfile",
            "-Command",
            (
                "Start-Process powershell "
                f"-ArgumentList '-NoExit', '-Command', 'wsl -d Ubuntu bash -lc \"{sim_command}\"'"
            ),
        ]
    )


def main() -> None:
    print("Starting University of Cyprus simulation...")
    print("drone1 will use mission UDP 14550, map UDP 14551, and QGroundControl UDP 14555")
    print("drone2 will use mission UDP 14560, map UDP 14561, and QGroundControl UDP 14556")
    print("drone3 will use mission UDP 14570, map UDP 14571, and QGroundControl UDP 14557")

    for drone in DRONE_CONFIGS:
        print(f"Launching {drone['name']}...")
        start_simulator(drone)

    print("\nOpen QGroundControl and make sure it listens on UDP 14555, 14556, and 14557.")
    print("Then run:")
    print("python3 three_drone_building_search.py --seed 7 --scan-delay 2 --altitude 10")


if __name__ == "__main__":
    main()
