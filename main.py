"""
Todo:
- fix coordinate transformation issue
"""

import math, sensor, time, machine, image
import pymavminimal as pymav
from machine import I2C
from vl53l1x import VL53L1X
from pyb import Timer

# Setup UART
UART_BAUDRATE = 256000
uart = machine.UART(9, UART_BAUDRATE)
uart.init(UART_BAUDRATE, bits=8, parity=None, stop=1)

# Setup TOF sensor
tof = VL53L1X(I2C(2))

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

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True) # doesn't seem to do anything
sensor.set_auto_exposure(False, exposure_us=500)
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
sensor.skip_frames(time=200)

# QVGA Camera properties
sensor_pixel_h = .00175 # mm
sensor_pixel_w = .00175 # mm
qvga_sensor_focal_len_avg_px = 450 # px
qvga_sensor_focal_length_mm = qvga_sensor_focal_len_avg_px * sensor_pixel_h
qvga_x_res = 320
qvga_y_res = 240
qvga_sensor_w_mm = sensor_pixel_w * qvga_x_res
qvga_sensor_h_mm = sensor_pixel_h * qvga_y_res
qvga_f_x = (qvga_sensor_focal_length_mm / qvga_sensor_w_mm) * qvga_x_res
qvga_f_y = (qvga_sensor_focal_length_mm / qvga_sensor_h_mm) * qvga_y_res
qvga_c_x = qvga_x_res / 2
qvga_c_y = qvga_y_res / 2
QVGA_SEARCH_IMG_AREA = qvga_x_res * qvga_y_res

# Target info
valid_tag_ids = {
    0: 16,  # tag black border size in mm
    1: 32,
    2: 64,
    3: 128,
}

tag_offsets_mm = {
    0: (0, 0, 0),
    1: (-30, -10, 0),
    2: (-10, -70, 0),
    3: (110, -30, 0),
}

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

def get_target_center_FRD_mm(tag):
    tag_size = valid_tag_ids[tag.id]

    # Euler angles in radians
    rx = tag.x_rotation
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
    cx += translation_to_mm(tag.x_translation, tag_size)
    cy += translation_to_mm(tag.y_translation, tag_size)
    cz += translation_to_mm(tag.z_translation, tag_size)

    print(f"COR x{cx}, y{cy}, z{cz}")

    # Convert camera frame to FRD
    frd_x_mm = cx
    frd_y_mm = cy
    frd_z_mm = -cz
    dist_mm = math.sqrt(
        translation_to_mm(tag.x_translation, tag_size) ** 2
        + translation_to_mm(tag.y_translation, tag_size) ** 2
        + translation_to_mm(tag.z_translation, tag_size) ** 2
    )
    return frd_x_mm, frd_y_mm, frd_z_mm, dist_mm

def send_landing_target(target_num, x, y, z, distance):
    led_fail.on()
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
    led_fail.off()


def perform_fine_search(img):
    blob_img = img.copy()
    blob_img.laplacian(3, sharpen=True, threshold=True, offset = -15)
    blob_img.midpoint(3, threshold=False).binary([(0,5)], invert=True)

    tag_list = []
    try:
        for blob in blob_img.find_blobs([(250,255)], pixels_threshold=360, merge=False,
            x_stride=4, y_stride=4, invert=False
        ):
            blob_area = blob.w() * blob.h()
            area_ratio = blob_area / QVGA_SEARCH_IMG_AREA

            if area_ratio > 0.90:
                continue  # Skip this blob
            w = min(max(int(blob.w() * 1.5), 10), 160)
            h = min(max(int(blob.h() * 1.5), 10), 120)
            x = min(max(int(blob.x() + ((blob.w()-w) / 2)), 0), img.width() - 1)
            y = min(max(int(blob.y() + ((blob.h()-h) / 2)), 0), img.height() - 1)
            roi = (x, y, w, h)

            tag_list.extend(img.find_apriltags(roi=roi, families=image.TAG36H11,
                fx=qvga_f_x,
                fy=qvga_f_y,
                cx=qvga_c_x-roi[0],
                cy=qvga_c_y-roi[1]
            ))
    except (MemoryError):
        pass
    return tag_list

def perform_coarse_search(img, coarse_scale_x, coarse_scale_y):
    coarse_img = img.crop(x_scale = coarse_scale_x, y_scale = coarse_scale_y, hint = image.BILINEAR, copy=True)
    tag_list = coarse_img.find_apriltags(
        fx=qvga_f_x * coarse_scale_x,
        fy=qvga_f_y * coarse_scale_y,
        cx=qvga_c_x * coarse_scale_x,
        cy=qvga_c_y * coarse_scale_y)
    return tag_list

def find_apriltag_optimized(img):
    scale_x = scale_y = .25
    tag_list = perform_coarse_search(img, scale_x, scale_y)
    if not tag_list:
        scale_x = scale_y = .5
        tag_list = perform_coarse_search(img, scale_x, scale_y)
    if not tag_list:
        scale_x = scale_y = 1
        tag_list = perform_fine_search(img)
    return tag_list, scale_x, scale_y

def draw_apriltags(img, tag_list, scale_x, scale_y):
    scaling_factor_x = 1/scale_x
    scaling_factor_y = 1/scale_y
    for tag in tag_list:
        img.draw_rectangle((
            int(tag.rect[0]*scaling_factor_x),
            int(tag.rect[1]*scaling_factor_y),
            int(tag.rect[2]*scaling_factor_x),
            int(tag.rect[3]*scaling_factor_y),
        ))
        img.draw_cross(int(tag.cx*scaling_factor_x), int(tag.cy*scaling_factor_y))


def translation_to_mm(translation, tag_size):
    # translation is in decimeters...
    return ((translation * 100) * tag_size) / 210

def find_distance(tag):
    return math.sqrt(
        translation_to_mm(tag.x_translation, valid_tag_ids[tag.id])**2 +
        translation_to_mm(tag.y_translation, valid_tag_ids[tag.id])**2 +
        translation_to_mm(tag.z_translation, valid_tag_ids[tag.id])**2
    )

# Main Loop
clock = time.clock()
last_hb_time = time.ticks_ms()
i = 0
while True:
    i+=1
    try:
        clock.tick()

        if send_heartbeat_flag:
            send_heartbeat_flag = False
            send_heartbeat()

        img = sensor.snapshot()
        # img.draw_cross(int(qvga_c_x), int(qvga_c_y))
        tag_list, scale_x, scale_y = find_apriltag_optimized(img)
        tag_list = sorted(tag_list, key=lambda x: x.w * x.h, reverse=True)
        if tag_list and (tag_list[0].id in valid_tag_ids):
            draw_apriltags(img, tag_list, scale_x, scale_y)
            tag = tag_list[0]
            x, y, z, distance = get_target_center_FRD_mm(tag)
            send_landing_target(tag.id, x, y, z, distance)
        machine.idle()
    except (MemoryError):
        print("Loop", i, "mem error")
