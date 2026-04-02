from pymavlink import mavutil
import time

m = mavutil.mavlink_connection("udp:127.0.0.1:14550", source_system=250)
hb = m.wait_heartbeat(timeout=20)
print(f"Connected to sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()}")

m.target_system = hb.get_srcSystem()
m.target_component = hb.get_srcComponent()

mode_id = m.mode_mapping()["GUIDED"]
m.mav.set_mode_send(
    m.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
time.sleep(2)

print("Arming...")
m.mav.command_long_send(
    m.target_system,
    m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
time.sleep(3)

print("Takeoff to 10 meters...")
m.mav.command_long_send(
    m.target_system,
    m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, 10
)

start = time.time()
while time.time() - start < 15:
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    if msg:
        alt = msg.relative_alt / 1000.0
        print(f"alt={alt:.1f}m")
