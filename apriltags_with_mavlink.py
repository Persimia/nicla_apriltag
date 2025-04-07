import math
import sensor
import struct
import time
import machine

from machine import I2C
from vl53l1x import VL53L1X

tof = VL53L1X(I2C(2))

UART_BAUDRATE = 115200
MAV_system_id = 1
MAV_component_id = 197 # Visual inertial odometry
packet_sequence = 0

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

valid_tag_ids = {
    0: 92.71,  # 8.5" x 11" tag black border size in mm
    1: 92.71,  # 8.5" x 11" tag black border size in mm
    2: 92.71,  # 8.5" x 11" tag black border size in mm
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

# Link Setup
uart = machine.UART(9, UART_BAUDRATE, timeout_char=1000)


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


MAV_LANDING_TARGET_message_id = 149
MAV_LANDING_TARGET_min_distance = 1 / 100  # in meters
MAV_LANDING_TARGET_max_distance = 10000 / 100  # in meters
MAV_LANDING_TARGET_frame = 8  # MAV_FRAME_BODY_NED
MAV_LANDING_TARGET_extra_crc = 200


# http://mavlink.org/messages/common#LANDING_TARGET
# https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_landing_target.h
def send_landing_target_packet(tag, dist_mm, w, h):
    global packet_sequence
    temp = struct.pack(
        "<qfffffbb",
        0,
        ((tag.cx / w) - 0.5) * h_fov,
        ((tag.cy / h) - 0.5) * v_fov,
        min(
            max(dist_mm * 0.001, MAV_LANDING_TARGET_min_distance),
            MAV_LANDING_TARGET_max_distance,
        ),
        0.0,
        0.0,
        0,
        MAV_LANDING_TARGET_frame,
    )
    temp = struct.pack(
        "<bbbbb30s",
        30,
        packet_sequence & 0xFF,
        MAV_system_id,
        MAV_component_id,
        MAV_LANDING_TARGET_message_id,
        temp,
    )
    temp = struct.pack(
        "<b35sh", 0xFE, temp, checksum(temp, MAV_LANDING_TARGET_extra_crc)
    )
    packet_sequence += 1
    uart.write(temp)


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


# Main Loop

clock = time.clock()
while True:
    clock.tick()
    img = sensor.snapshot()
    tags = sorted(
        img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y),
        key=lambda x: x.w * x.h,
        reverse=True,
    )
    target_found = False
    if tags and (tags[0].id in valid_tag_ids):
        target_found = True
        tag_size = valid_tag_ids[tags[0].id]
        dist_mm = math.sqrt(
            translation_to_mm(tags[0].x_translation, tag_size) ** 2
            + translation_to_mm(tags[0].y_translation, tag_size) ** 2
            + translation_to_mm(tags[0].z_translation, tag_size) ** 2
        )
        send_landing_target_packet(tags[0], dist_mm, img.width(), img.height())
        img.draw_rectangle(tags[0].rect)
        img.draw_cross(tags[0].cx, tags[0].cy)
        true_dist = tof.read()
        print("ATd %f mm, Td %f mm - FPS %f" % (dist_mm, true_dist, clock.fps()))
    else:
        print("FPS %f" % clock.fps())

    update_led(target_found)
