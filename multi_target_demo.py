import math, sensor, time, machine
import pymavminimal as pymav
from machine import I2C
from vl53l1x import VL53L1X
from pyb import Timer

# Setup UART
UART_BAUDRATE = 256000
uart = machine.UART(9, UART_BAUDRATE)
uart.init(UART_BAUDRATE, bits=8, parity=None, stop=1)

# Set up MAVLink
mavlink = pymav.MAVLink(None)
mavlink.srcSystem = 1
mavlink.srcComponent = 197
MAV_system_id = 1
MAV_component_id = 197 # Visual inertial odometry
packet_sequence = 0
MAV_LANDING_TARGET_message_id = 149
MAV_LANDING_TARGET_min_distance = 1 / 100  # in meters
MAV_LANDING_TARGET_max_distance = 10000 / 100  # in meters
MAV_LANDING_TARGET_frame = 8  # MAV_FRAME_BODY_NED
MAV_LANDING_TARGET_extra_crc = 200
MAV_FRAME_BODY_FRD = 12
LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2

# Setup TOF sensor
tof = VL53L1X(I2C(2))

# Setup Camera Info
sensor_num_pixels_w = 1616 # 1600
sensor_num_pixels_h = 1232 # 1200
x_res = 160  # QQVGA pixels
y_res = 120  # QQVGA pixels
sensor_focal_len_avg_px = 450 #px
sensor_pixel_h = .00175 # mm
sensor_pixel_w = .00175 # mm
sensor_focal_length_mm = sensor_focal_len_avg_px * sensor_pixel_h
sensor_w_mm = sensor_pixel_w*x_res  # For GC2145 sensor
sensor_h_mm = sensor_pixel_h*y_res  # For GC2145 sensor

# Target info
valid_tag_ids = {
    0: 150,  # 8.5" x 11" tag black border size in mm
    1: 77.5,  # 8.5" x 11" tag black border size in mm
    2: 34.5,  # 8.5" x 11" tag black border size in mm
}

tag_offsets_mm = {
    0: (-21.25, 104.25, 0),
    1: (-67, -23.5, 0),
    2: (0, 0, 0),
}


# Camera Setup
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_whitebal(False)
f_x = (sensor_focal_length_mm / sensor_w_mm) * x_res
f_y = (sensor_focal_length_mm / sensor_h_mm) * y_res
c_x = x_res / 2
c_y = y_res / 2
h_fov = 2 * math.atan((sensor_w_mm / 2) / sensor_focal_length_mm)
v_fov = 2 * math.atan((sensor_h_mm / 2) / sensor_focal_length_mm)


def translation_to_mm(translation, tag_size):
    # translation is in decimeters...
    return ((translation * 100) * tag_size) / 210

# https://github.com/mavlink/c_library_v1/blob/master/checksum.h
def checksum(data, extra):
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output

# Flag for heartbeat
send_heartbeat_flag = False

# Timer callback sets a flag
def timer_callback(t):
    global send_heartbeat_flag
    send_heartbeat_flag = True

# Set up timer to trigger every second
heartbeat_timer = Timer(4, freq=1, callback=timer_callback)

def send_heartbeat():
    led_success.on()
    try:
        msg = mavlink.heartbeat_encode(
            pymav.MAV_TYPE_ONBOARD_CONTROLLER,
            8, 0, 0, 4
        )
        packet = msg.pack(mavlink)
        uart.write(packet)
        while not uart.txdone():
            pass
    except Exception as e:
        print("Error sending heartbeat:", e)
    finally:
        led_success.off()

# LED control
led_success = machine.LED("LED_GREEN")
led_fail = machine.LED("LED_RED")
led_counter = 0


def update_led(target_found):
    global led_counter

    if target_found:
        led = led_success
        led_fail.off()
    else:
        led = led_fail
        led_success.off()

    if led_counter % 4 == 0:
        led.toggle()

    led_counter += 1

def get_target_center_FRD_mm(tag):
    tag_size = valid_tag_ids[tags[0].id]

    # Euler angles in radians
    rx = tag.x_rotation - math.pi
    ry = tag.y_rotation
    rz = tag.z_rotation

    # Build rotation matrix from Euler angles (assuming XYZ rotation order)
    # First compute cosines and sines
    cx = math.cos(rx)
    sx = math.sin(rx)
    cy = math.cos(ry)
    sy = math.sin(ry)
    cz = math.cos(rz)
    sz = math.sin(rz)

    # Rotation matrix R = Rz * Ry * Rx
    R = [
        [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
        [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
        [-sy,   cy*sx,             cy*cx]
    ]

    # Offset from tag to center (in tag's frame)
    dx, dy, dz = tag_offsets_mm[tag.id]

    # Transform to camera space: R' * offset
    cx = R[0][0]*dx + R[1][0]*dy + R[2][0]*dz
    cy = R[0][1]*dx + R[1][1]*dy + R[2][1]*dz
    cz = R[0][2]*dx + R[1][2]*dy + R[2][2]*dz

    # print(f"ROT x{rx}, y{ry}, z{rz}")
    # print(f"OFF x{cx}, y{cy}, z{cz}")
    # print(f"IMG x{translation_to_mm(tags[0].x_translation, tag_size)}, y{translation_to_mm(tags[0].y_translation, tag_size)}, z{translation_to_mm(tags[0].z_translation, tag_size)}")

    # Add translation from tag to camera
    cx += translation_to_mm(tags[0].x_translation, tag_size)
    cy += translation_to_mm(tags[0].y_translation, tag_size)
    cz += translation_to_mm(tags[0].z_translation, tag_size)

    print(f"COR x{cx}, y{cy}, z{cz}")

    # Convert camera frame to FRD
    frd_x_mm = cx
    frd_y_mm = cy
    frd_z_mm = -cz

    dist_mm = math.sqrt(
        translation_to_mm(tags[0].x_translation, tag_size) ** 2
        + translation_to_mm(tags[0].y_translation, tag_size) ** 2
        + translation_to_mm(tags[0].z_translation, tag_size) ** 2
    )

    return frd_x_mm, frd_y_mm, frd_z_mm, dist_mm


def send_landing_target(target_num, x, y, z, distance):
    led_fail.on()
    # try:
    msg = mavlink.landing_target_encode(
        time.ticks_us(),
        target_num,
        MAV_FRAME_BODY_FRD, # MAV_FRAME_BODY_FRD (only supported frame for ardupilot)
        0,
        0,
        distance,
        0,
        0,
        x = x,
        y = y,
        z = z,
        q = [1,0,0,0], # unused
        type = pymav.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        position_valid = True, # chooses between relative and absolute pos
    )

    packet = msg.pack(mavlink)
    uart.write(packet)
    while not uart.txdone():
        pass
    # except Exception as e:
    #     print("Error sending landing target:", e)
    # finally:
    led_fail.off()

# Main Loop
clock = time.clock()
last_hb_time = time.ticks_ms()

# Main loop does the actual sending
while True:
    clock.tick()

    if send_heartbeat_flag:
        send_heartbeat_flag = False
        send_heartbeat()

    img = sensor.snapshot()
    tags = sorted(
        img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y),
        key=lambda x: x.w * x.h,
        reverse=True,
    )
    target_found = False
    if tags and (tags[0].id in valid_tag_ids):
        target_found = True

        x, y, z, distance = get_target_center_FRD_mm(tags[0])
        send_landing_target(tags[0].id, x, y, z, distance)

        img.draw_rectangle(tags[0].rect)
        img.draw_cross(tags[0].cx, tags[0].cy)
        true_dist = tof.read()

    else:
        # print("FPS %f" % clock.fps())
        pass


    machine.idle()
