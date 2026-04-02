from pymavlink import mavutil

ports = {
    "drone1": "tcp:127.0.0.1:5760",
    "drone2": "tcp:127.0.0.1:5770",
}

for name, port in ports.items():
    print(f"\nConnecting to {name} on {port} ...")
    m = mavutil.mavlink_connection(port)
    msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=20)
    print(f"heartbeat message: {msg}")
    if msg:
        print(f"{name}: sysid={msg.get_srcSystem()} compid={msg.get_srcComponent()}")
