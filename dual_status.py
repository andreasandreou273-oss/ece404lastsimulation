from pymavlink import mavutil
import time

ports = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}

connections = {}

for name, port in ports.items():
    print(f"Connecting to {name} on {port} ...")
    m = mavutil.mavlink_connection(port, source_system=250)
    hb = m.wait_heartbeat(timeout=20)
    print(f"{name}: sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()}")
    m.target_system = hb.get_srcSystem()
    m.target_component = hb.get_srcComponent()
    connections[name] = m

print("Reading both drones. Press Ctrl+C to stop.")

while True:
    for name, m in connections.items():
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            print(f"{name}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")
    time.sleep(1)
