from pymavlink import mavutil
import time

ports = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}

connections = {}

for name, port in ports.items():
    print(f"Connecting to {name} on {port} ...")
    m = mavutil.mavlink_connection(port)
    hb = m.wait_heartbeat(timeout=20)
    print(f"{name} connected with sysid={hb.get_srcSystem()}")
    connections[name] = m

print("Reading positions. Press Ctrl+C to stop.")

while True:
    for name, m in connections.items():
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            sysid = msg.get_srcSystem()
            print(f"{name} (sysid={sysid}): lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")
    time.sleep(1)
