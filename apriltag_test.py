import sensor, image, time
import machine
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink import mavutil

# Initialize Camera
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # AprilTags work best in grayscale
sensor.set_framesize(sensor.QQVGA)      # Lower resolution to save memory
sensor.set_vflip(False)
sensor.set_hmirror(False)
sensor.skip_frames(time=2000)

clock = time.clock()
redLED = machine.LED("LED_RED")
greenLED = machine.LED("LED_GREEN")
blueLED = machine.LED("LED_BLUE")

# AprilTag detection settings
tag_families = image.TAG36H11  # Use only one family to save memory

# Camera Intrinsics (adjust based on calibration)
f_x = 100  # Estimated focal length in pixels
f_y = 100
c_x = sensor.width() / 2
c_y = sensor.height() / 2
APRILTAG_SIZE_MM = 88
scale_factor = APRILTAG_SIZE_MM

uart = machine.UART(9)
uart.init(baudrate = 115200, bits = 8, parity = None, stop = 1)  # Adjust pins for your board

# MAVLink constants
MAV_SYS_ID = 1
MAV_COMP_ID = 195  # Typically 195 for vision system
TARGET_SYS_ID = 1
TARGET_COMP_ID = 1

# Create MAVLink connection
mav = mavutil.mavlink_connection(
    device=uart,
    source_system=1,
    source_component=195,
    dialect='common'
)

interval = 1000  # milliseconds
next_heartbeat = time.ticks_add(time.ticks_ms(), interval)
def send_heartbeat():
    mav.mav.heartbeat_send(
        type=mavlink2.MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavlink2.MAV_STATE_ACTIVE,
        mavlink_version=3
    )

def send_landing_target(timestamp_us, target_num, frame,
    angle_x_rad, angle_y_rad, distance_m,
    size_x, size_y,
    x_m, y_m, z_m,
    q, target_type, position_valid,
):
    mav.mav.landing_target_send(
        time_usec = timestamp_us,
        target_num = target_num,
        frame = frame,
        angle_x = angle_x_rad,
        angle_y = angle_y_rad,
        distance = distance_m,
        size_x = size_x,
        size_y = size_y,
        x = x_m,
        y = y_m,
        z = z_m,
        q = q,
        type = target_type,
        position_valid = position_valid
    )

while True:
    clock.tick()
    if time.ticks_diff(time.ticks_ms(), next_heartbeat) >= 0:
        send_heartbeat()
        next_heartbeat = time.ticks_add(next_heartbeat, interval)

    img = sensor.snapshot()
    tags = img.find_apriltags(families=tag_families)

    for tag in tags:  # Detect tags
        # Pose estimation (try-except to handle memory errors)
        try:
            translation = [
                tag.x_translation * scale_factor,
                tag.y_translation * scale_factor,
                tag.z_translation * scale_factor
            ]
            rotation = [
                tag.x_rotation,
                tag.y_rotation,
                tag.z_rotation
            ]

            print("Tag ID:", tag.id)
            print("Translation:", translation)
            print("Rotation:", rotation)
            print("-------------------")

            now_us = int(time.ticks_us())
            send_landing_target(now_us, angle_x=0.1, angle_y=0.05, distance=1.5)

            redLED.off()
            greenLED.on()

        except MemoryError:
            print("Memory Error: Pose estimation skipped!")
    if not tags:
        greenLED.off()
        redLED.on()

    print("FPS:", clock.fps())  # Show real-time performance
