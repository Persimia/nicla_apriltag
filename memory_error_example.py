import math, sensor, time, machine, image
import gc

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
        translation_to_mm(tag.x_translation, 100)**2 +
        translation_to_mm(tag.y_translation, 100)**2 +
        translation_to_mm(tag.z_translation, 100)**2
    )

# Main Loop
clock = time.clock()
last_hb_time = time.ticks_ms()
i = 0
while True:
    clock.tick()

    img = sensor.snapshot()
    tag_list, scale_x, scale_y = find_apriltag_optimized(img)
    if tag_list:
        draw_apriltags(img, tag_list, scale_x, scale_y)
    machine.idle()

    i+=1
    print("Loop", i, "- Free mem:", gc.mem_free())
