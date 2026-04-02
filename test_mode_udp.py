from pymavlink import mavutil
import time

print("Connecting to drone2 UDP stream...")
m = mavutil.mavlink_connection("udp:127.0.0.1:14560", source_system=250)
hb = m.wait_heartbeat(timeout=20)

if hb is None:
    print("No heartbeat received")
    raise SystemExit(1)

sysid = hb.get_srcSystem()
compid = hb.get_srcComponent()
print(f"Heartbeat from sysid={sysid}, compid={compid}")

# make sure target ids are correct
m.target_system = sysid
m.target_component = compid

mode_name = "GUIDED"
mode_map = m.mode_mapping()
print("Mode map:", mode_map)

if not mode_map or mode_name not in mode_map:
    print("Could not find GUIDED in mode mapping")
    raise SystemExit(1)

mode_id = mode_map[mode_name]
print(f"Sending {mode_name} (mode_id={mode_id}) to sysid={sysid}")

m.mav.set_mode_send(
    m.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

print("Command sent. Waiting 8 seconds for any reply...")
start = time.time()
while time.time() - start < 8:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg:
        print(msg.get_type(), msg.get_srcSystem(), msg.get_srcComponent())
