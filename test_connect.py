from pymavlink import mavutil

ports = [
    "udp:127.0.0.1:14550",
    "udp:127.0.0.1:14560",
]

for port in ports:
    print(f"Connecting to {port} ...")
    m = mavutil.mavlink_connection(port)
    m.wait_heartbeat(timeout=20)
    print(f"Connected on {port} with SYSID={m.target_system}")
