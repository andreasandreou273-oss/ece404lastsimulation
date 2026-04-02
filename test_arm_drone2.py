from pymavlink import mavutil
import time

m = mavutil.mavlink_connection("udp:127.0.0.1:14560", source_system=250)
hb = m.wait_heartbeat(timeout=20)
print(f"Connected to sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()}")

m.target_system = hb.get_srcSystem()
m.target_component = hb.get_srcComponent()

# put in GUIDED first
mode_id = m.mode_mapping()["GUIDED"]
m.mav.set_mode_send(
    m.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
time.sleep(2)

print("Sending arm command...")
m.mav.command_long_send(
    m.target_system,
    m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

start = time.time()
while time.time() - start < 10:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg:
        print(msg.get_type(), msg.get_srcSystem(), msg.get_srcComponent())
