from pymavlink import mavutil
import time

# Connect to drone2 on the sysid-separated UDP stream
m = mavutil.mavlink_connection("udp:127.0.0.1:14560")
hb = m.wait_heartbeat(timeout=20)
print(f"Connected to sysid={hb.get_srcSystem()}")

# Ask for GUIDED mode
mode = "GUIDED"
mode_id = m.mode_mapping().get(mode)

if mode_id is None:
    print("Could not find GUIDED mode")
    raise SystemExit(1)

m.mav.set_mode_send(
    m.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

print("GUIDED mode command sent, waiting for ACK/messages...")

start = time.time()
while time.time() - start < 8:
    msg = m.recv_match(blocking=True, timeout=2)
    if msg:
        print(msg)
