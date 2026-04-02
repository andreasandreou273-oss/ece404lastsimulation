from pymavlink import mavutil
import time
import math

DRONE1_PORT = "udp:127.0.0.1:14550"
DRONE2_PORT = "udp:127.0.0.1:14560"

def connect(name, port):
    m = mavutil.mavlink_connection(port, source_system=250)
    hb = m.wait_heartbeat(timeout=20)
    m.target_system = hb.get_srcSystem()
    m.target_component = hb.get_srcComponent()
    print(f"{name} connected: sysid={m.target_system}, compid={m.target_component}")
    return m

def get_pos(m):
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
    if not msg:
        return None
    return {
        "lat": msg.lat / 1e7,
        "lon": msg.lon / 1e7,
        "alt": msg.relative_alt / 1000.0,
    }

def distance_m(a, b):
    dx = (b["lon"] - a["lon"]) * 111320 * math.cos(math.radians((a["lat"] + b["lat"]) / 2))
    dy = (b["lat"] - a["lat"]) * 110540
    return math.sqrt(dx * dx + dy * dy)

def set_guided(m):
    mode_id = m.mode_mapping()["GUIDED"]
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(1)

def goto(m, lat, lon, alt):
    m.mav.set_position_target_global_int_send(
        0,
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

drone1 = connect("drone1", DRONE1_PORT)
drone2 = connect("drone2", DRONE2_PORT)

set_guided(drone1)
set_guided(drone2)

print("Reading starting positions...")
p1 = get_pos(drone1)
p2 = get_pos(drone2)

if not p1 or not p2:
    raise RuntimeError("Could not read starting positions from both drones")

print("Start:")
print("drone1", p1)
print("drone2", p2)

# Move drone1 about 8–10 meters north
target1 = {
    "lat": p1["lat"] + 0.00008,
    "lon": p1["lon"],
    "alt": max(p1["alt"], 10.0),
}

print(f"Sending drone1 to new point: {target1}")
goto(drone1, target1["lat"], target1["lon"], target1["alt"])

# Wait for drone1 to get close enough
while True:
    p1 = get_pos(drone1)
    if p1:
        d1 = distance_m(p1, target1)
        print(f"drone1 now at lat={p1['lat']:.6f}, lon={p1['lon']:.6f}, alt={p1['alt']:.1f}m, dist_to_target={d1:.1f}m")
        if d1 < 2.5:
            print("Drone1 reached target.")
            break
    time.sleep(1)

# "Message" event from drone1 to drone2
print("MESSAGE: Drone1 -> Drone2 : Come to my location")

# Send drone2 to drone1's current location
follow_target = p1
print(f"Sending drone2 to drone1 location: {follow_target}")
goto(drone2, follow_target["lat"], follow_target["lon"], max(follow_target["alt"], 10.0))

# Monitor both until drone2 gets close
while True:
    p1_new = get_pos(drone1)
    p2_new = get_pos(drone2)

    if p1_new:
        p1 = p1_new
    if p2_new:
        p2 = p2_new

    d2 = distance_m(p2, p1)
    print(
        f"drone1 lat={p1['lat']:.6f}, lon={p1['lon']:.6f}, alt={p1['alt']:.1f}m | "
        f"drone2 lat={p2['lat']:.6f}, lon={p2['lon']:.6f}, alt={p2['alt']:.1f}m | "
        f"separation={d2:.1f}m"
    )

    if d2 < 3.0:
        print("Drone2 reached Drone1 location.")
        break

    time.sleep(1)
