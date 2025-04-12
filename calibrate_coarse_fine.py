import sensor, image, time, math

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True) # doesn't seem to do anything
sensor.set_auto_exposure(False, exposure_us=500)
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
sensor.skip_frames(time=200)

clock = time.clock()

COARSE_SCALE_X = .5
COARSE_SCALE_Y = .5

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
    0: 150,  # 8.5" x 11" tag black border size in mm
    1: 77.5,  # 8.5" x 11" tag black border size in mm
    2: 34.5,  # 8.5" x 11" tag black border size in mm
}

tag_offsets_mm = {
    0: (-21.25, 104.25, 0),
    1: (-67, -23.5, 0),
    2: (0, 0, 0),
}


def set_sensor_for_area_search():
    sensor.set_framesize(sensor.QVGA)
    # sensor.skip_frames(1)

def perform_area_search(img):
    blob_img = img.copy()
    blob_img.laplacian(3, sharpen=True, threshold=True, offset = -15)
    blob_img.midpoint(3, threshold=False).binary([(0,5)], invert=True)

    tag_list = []

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

        try:
            tag_list.extend(img.find_apriltags(roi=roi, families=image.TAG36H11,
                fx=qvga_f_x,
                fy=qvga_f_y,
                cx=qvga_c_x-roi[0],
                cy=qvga_c_y-roi[1]
            ))
        except (MemoryError):
            pass
    return tag_list

def scale_coarse_results(tag_list):
    pass

def perform_coarse_search(img):
    coarse_img = img.crop(x_scale = .5, y_scale = .5, hint = image.BILINEAR, copy=True)
    tag_list = coarse_img.find_apriltags(
        fx=qvga_f_x * COARSE_SCALE_X,
        fy=qvga_f_y * COARSE_SCALE_Y,
        cx=qvga_c_x * COARSE_SCALE_X,
        cy=qvga_c_y * COARSE_SCALE_Y)
    return tag_list

def find_apriltag_optimized(img):
    tag_list = perform_coarse_search(img)
    if not tag_list:
        tag_list = perform_area_search(img)
        print("Fine")
    else:
        print("Coarse")
    draw_apriltags(img, tag_list)

def draw_apriltags(img, tag_list):
    for tag in tag_list:
        img.draw_rectangle(tag.rect)
        img.draw_cross(tag.cx, tag.cy)
        for c in tag.corners:
            img.draw_circle(c[0], c[1], 5)
        print("Tag:", tag.cx, tag.cy, tag.rotation, tag.id)

def translation_to_mm(translation, tag_size):
    # translation is in decimeters...
    return ((translation * 100) * tag_size) / 210

def find_distance(tag):
    return math.sqrt(
        translation_to_mm(tag.x_translation, valid_tag_ids[tag.id])**2 +
        translation_to_mm(tag.y_translation, valid_tag_ids[tag.id])**2 +
        translation_to_mm(tag.z_translation, valid_tag_ids[tag.id])**2
    )

while True:
    clock.tick()
    img = sensor.snapshot()
    coarse_tag_list = perform_coarse_search(img)
    fine_tag_list = perform_area_search(img)
    if coarse_tag_list:
        print(f"Coarse: {find_distance(coarse_tag_list[0])}")
    if fine_tag_list:
        print(f"Fine: {find_distance(fine_tag_list[0])}")


