from pymavlink import mavutil
import time

ports = {
    "drone1": "udp:127.0.0.1:14550",
    "drone2": "udp:127.0.0.1:14560",
}

for name, port in ports.items():
    print(f"\n--- {name} / {port} ---")
    m = mavutil.mavlink_connection(port)
    m.wait_heartbeat(timeout=20)
    print("heartbeat ok")

    start = time.time()
    seen = set()

    while time.time() - start < 10:
        msg = m.recv_match(blocking=False)
        if msg:
            t = msg.get_type()
            if t not in seen:
                seen.add(t)
                print(t)

    print(f"Total message types seen: {len(seen)}")
