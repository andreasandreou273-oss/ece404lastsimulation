from pymavlink import mavutil
import time

connections = {
    "drone1": "tcp:127.0.0.1:5760",
    "drone2": "tcp:127.0.0.1:5770",
}

GLOBAL_POSITION_INT_ID = 33  # MAVLink common message id

for name, endpoint in connections.items():
    print(f"\nConnecting to {name} on {endpoint} ...")
    m = mavutil.mavlink_connection(endpoint)
    m.wait_heartbeat(timeout=20)
    print("heartbeat ok")

    # Ask ArduPilot to stream GLOBAL_POSITION_INT at 2 Hz
    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        GLOBAL_POSITION_INT_ID,
        500000,   # microseconds between messages = 2 Hz
        0, 0, 0, 0, 0
    )

    # wait briefly for ack / stream start
    time.sleep(2)

    got_any = False
    start = time.time()
    while time.time() - start < 10:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            got_any = True
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            print(f"{name}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")
            break

    if not got_any:
        print(f"{name}: no GLOBAL_POSITION_INT received")
