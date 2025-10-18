from pymavlink import mavutil
import time

def connect(connection_string):
    master = mavutil.mavlink_connection(connection_string)
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system {master.target_system} component {master.target_component}")
    return master

def wait_for_gps(master):
    print("Waiting for GPS fix...")
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if msg and hasattr(msg, 'fix_type') and msg.fix_type >= 3:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            print(f"GPS fix obtained: lat={lat}, lon={lon}")
            return lat, lon
        print("No GPS fix yet, retrying...")
        time.sleep(1)

def disable_arming_checks(master):
    print("Disabling arming checks...")
    params = {
        'ARMING_CHECK': 0,
        'FS_THR_ENABLE': 0,
        'FS_BATT_ENABLE': 0
    }
    for param, value in params.items():
        master.param_set_send(param, float(value))
        time.sleep(0.1)

def set_airspeed_params(master):
    print("Setting airspeed simulation params...")
    master.param_set_send('ARSPD_USE', 1)
    time.sleep(0.1)
    master.param_set_send('ARSPD_TYPE', 1)
    time.sleep(0.1)
    master.param_set_send('SIM_ARSPD_RATIO', 1.9)
    time.sleep(0.1)
    master.param_set_send('SIM_ARSPD_OFS', 2013)
    time.sleep(0.1)

def set_mode(master, mode_str):
    mode_id = master.mode_mapping()[mode_str]
    master.set_mode(mode_id)
    print(f"Requested mode change to {mode_str}")
    time.sleep(2)

def arm_vehicle(master):
    master.arducopter_arm()
    print("Arming...")
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("Vehicle armed")
            break
        time.sleep(1)

def takeoff(master, altitude):
    print(f"Taking off to {altitude}m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if pos:
            alt = pos.relative_alt / 1000.0
            print(f"Altitude: {alt:.1f}m")
            if alt >= altitude - 1.0:
                print("Reached target altitude")
                break
        time.sleep(1)

def upload_square_mission(master, lat, lon, size=0.001, altitude=50):
    print("Uploading mission...")
    master.waypoint_clear_all_send()
    time.sleep(1)

    points = [
        (lat, lon),
        (lat, lon + size),
        (lat + size, lon + size),
        (lat + size, lon),
        (lat, lon)
    ]

    mission_items = []
    for seq, (p_lat, p_lon) in enumerate(points):
        mission_items.append(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                master.target_system,
                master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0,
                int(p_lat * 1e7),
                int(p_lon * 1e7),
                altitude
            )
        )

    master.mav.mission_count_send(master.target_system, master.target_component, len(mission_items))

    for i in range(len(mission_items)):
        while True:
            req = master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=10)
            if req and req.seq == i:
                master.mav.send(mission_items[i])
                print(f"Sent waypoint {i}")
                break

    while True:
        ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
        if ack:
            if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("Mission upload complete")
            else:
                print(f"Mission upload failed: {ack.type}")
            break

def start_mission(master):
    set_mode(master, 'AUTO')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Mission started")

def main():
    master = connect('udp:127.0.0.1:14550')

    lat, lon = wait_for_gps(master)
    disable_arming_checks(master)
    set_airspeed_params(master)

    set_mode(master, 'GUIDED')
    arm_vehicle(master)
    takeoff(master, 30)

    set_mode(master, 'CRUISE')
    upload_square_mission(master, lat, lon, 0.0001, 30)

    start_mission(master)

    print("Mission running for 60 seconds...")
    time.sleep(60)

    set_mode(master, 'QSTABILIZE')
    set_mode(master, 'LAND')
    print("Landing complete")

if __name__ == "__main__":
    main()
