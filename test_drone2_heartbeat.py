from pymavlink import mavutil
import time

# outbound local port 14561, remote MAVProxy output 14560
m = mavutil.mavlink_connection('udpin:0.0.0.0:14561')
print("Waiting for heartbeat on local port 14561...")
hb = m.wait_heartbeat(timeout=20)
print(f"Got heartbeat from sysid={hb.get_srcSystem()}")

start = time.time()
while time.time() - start < 5:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg:
        print(msg.get_type(), msg.get_srcSystem(), msg.get_srcComponent())
