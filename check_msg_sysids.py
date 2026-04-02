from pymavlink import mavutil
import time

ports = {
    "port14550": "udp:127.0.0.1:14550",
    "port14560": "udp:127.0.0.1:14560",
}

for name, port in ports.items():
    print(f"\n--- checking {name} / {port} ---")
    m = mavutil.mavlink_connection(port)
    m.wait_heartbeat(timeout=20)
    start = time.time()
    count = 0

    while time.time() - start < 8 and count < 15:
        msg = m.recv_match(blocking=True, timeout=2)
        if msg:
            print(f"type={msg.get_type()} sysid={msg.get_srcSystem()} compid={msg.get_srcComponent()}")
            count += 1
