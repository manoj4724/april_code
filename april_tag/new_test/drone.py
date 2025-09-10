# drone.py
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from precision_land import PrecisionLander
from config import connection_string, baud_rate

# Flight parameters
TARGET_ALTITUDE = 3.0           # m
HOVER_AFTER_TAKEOFF = 8         # seconds
APPROACH_ALTITUDE = 2.5
ALIGN_ALTITUDE_1 = 2.0          # First hover & align
ALIGN_ALTITUDE_2 = 1.0          # Second hover & align
LAND_SPEED = 0.3                # m/s

SCAN_TIMEOUT = 15               # max time to find tag
HOVER_ALIGN_TIME = 5            # seconds at each align stage
CHECK_INTERVAL = 0.5
MIN_DETECTIONS_FOR_ALIGN = 3


def arm_and_takeoff(vehicle, target_altitude):
    print("?? Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True


    print(f"?? Taking off to {target_altitude}m...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"?? Altitude: {alt:.1f}m")
        if alt >= target_altitude * 0.95:
            break
        time.sleep(1)
    print("? Reached target altitude.")


def descend_to_and_hover(vehicle, target_alt, hover_time, lander, stage_name):
    print(f"?? Descending to {target_alt}m for {stage_name}...")
    target_loc = LocationGlobalRelative(
        vehicle.location.global_frame.lat,
        vehicle.location.global_frame.lon,
        target_alt
    )
    vehicle.simple_goto(target_loc, groundspeed=0.5)
    time.sleep(3)  # Allow time to reach

    start_time = time.time()
    while time.time() - start_time < hover_time:
        remaining = hover_time - (time.time() - start_time)
        print(f"?? {stage_name}: Hovering at {target_alt}m ({remaining:.1f}s left)", end="")

        if lander.is_target_acquired():
            print(" ? ? Tag seen, correcting position")
        else:
            print(" ? ?? No tag")

        time.sleep(CHECK_INTERVAL)

    print(f"? {stage_name}: Completed.")


def main():
    print("?? Connecting to vehicle...")
    try:
        vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    except Exception as e:
        print(f"? Failed to connect: {e}")
        return

    print(f"? Connected: {vehicle.version}")

    lander = PrecisionLander(vehicle)
    lander.start()
    time.sleep(2)

    try:
        # 1. Take off
        arm_and_takeoff(vehicle, TARGET_ALTITUDE)

        # 2. Hover after takeoff
        print(f"?? Hovering for {HOVER_AFTER_TAKEOFF} seconds...")
        time.sleep(HOVER_AFTER_TAKEOFF)
        print("? Post-takeoff hover complete.")

        # 3. Fly to landing point
        landing_point = LocationGlobalRelative(
            vehicle.location.global_frame.lat,
            vehicle.location.global_frame.lon,
            0
        )
        print("?? Flying to landing area...")
        vehicle.simple_goto(landing_point, groundspeed=5)
        time.sleep(8)

        # 4. Scan for tag
        print(f"?? Scanning for AprilTag from ~{APPROACH_ALTITUDE}m...")
        tag_found = False
        scan_start = time.time()
        while time.time() - scan_start < SCAN_TIMEOUT:
            if lander.is_target_acquired():
                print("? AprilTag acquired!")
                tag_found = True
                break
            print(f"? Searching... {int(time.time() - scan_start)}s/{SCAN_TIMEOUT}s")
            time.sleep(CHECK_INTERVAL)

        # 5. Alignment Phase 1: 2.0m
        descend_to_and_hover(
            vehicle=vehicle,
            target_alt=ALIGN_ALTITUDE_1,
            hover_time=HOVER_ALIGN_TIME,
            lander=lander,
            stage_name="ALIGN PHASE 1 (2m)"
        )

        # 6. Alignment Phase 2: 1.0m
        descend_to_and_hover(
            vehicle=vehicle,
            target_alt=ALIGN_ALTITUDE_2,
            hover_time=HOVER_ALIGN_TIME,
            lander=lander,
            stage_name="ALIGN PHASE 2 (1m)"
        )

        # 7. Final slow landing
        print("?? Starting final precision descent...")
        vehicle.parameters['LAND_SPEED'] = int(LAND_SPEED * 100)  # cm/s
        vehicle.mode = VehicleMode("LAND")

        while vehicle.armed:
            alt = vehicle.location.global_relative_frame.alt
            if alt is not None:
                print(f"?? Landing... Altitude: {alt:.2f}m")
            time.sleep(1)

        print("?? Landed successfully!")

    except Exception as e:
        print(f"? Error: {e}")
    finally:
        lander.stop()
        vehicle.close()
        print("?? Vehicle connection closed.")


if __name__ == "__main__":
    main()