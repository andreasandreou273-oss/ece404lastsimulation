from pymavlink import mavutil

ports = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}

for name, port in ports.items():
    print(f"Connecting to {name} on {port} ...")
    m = mavutil.mavlink_connection(port)
    hb = m.wait_heartbeat(timeout=20)
    print(f"{name}: source system from heartbeat = {hb.get_srcSystem()}")
