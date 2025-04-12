# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Find Small Apriltags
#
# This script shows off how to use blob tracking as a pre-filter to
# finding Apriltags in the image using blob tracking to find the
# area of where the tag is first and then calling find_apriltags
# on that blob.

# Note, this script works well assuming most parts of the image do not
# pass the thresholding test... otherwise, you don't get a distance
# benefit.

import sensor
import image
import time

W_LIMIT = 160
H_LIMIT = 120
DOWNSCALE_FACTOR = .5

# Set the thresholds to find a white object (i.e. tag border)
thresholds = [(250,255)]

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
IMG_AREA = 320*240
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True) # doesn't seem to do anything
sensor.set_auto_exposure(False,exposure_us=500)
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
sensor.skip_frames(time=200)  # increase this to let the auto methods run for longer

clock = time.clock()

tag_families = image.TAG36H11

def blob_merge_cb(b1, b2):
    print(b1.code())
    if b1.code() == b2.code():
        return False
    else:
        return True


while True:
    clock.tick()

    img = sensor.snapshot()
    blob_img = img.copy()
    blob_img.laplacian(3, sharpen=True, threshold=True, offset = -15)
    blob_img.midpoint(3, threshold=False).binary([(0,5)], invert=True)

    box_list = []
    tag_list = []

    for blob in blob_img.find_blobs(
        thresholds, pixels_threshold=360, merge=False,
        x_stride=4, y_stride=4, invert=False
    ):
        blob_area = blob.w() * blob.h()
        area_ratio = blob_area / IMG_AREA

        if area_ratio > 0.90:
            continue  # Skip this blob

        # Next we look for a tag in an ROI that's bigger than the blob.
        w = min(max(int(blob.w() * 1.5), 10), W_LIMIT)  # Not too small, not too big.
        h = min(max(int(blob.h() * 1.5), 10), H_LIMIT)  # Not too small, not too big.
        x = min(max(int(blob.x() + ((blob.w()-w) / 2)), 0), img.width() - 1)
        y = min(max(int(blob.y() + ((blob.h()-h) / 2)), 0), img.height() - 1)

        roi = (x, y, w, h)
        box_list.append(roi)  # We'll draw these later.

        # Since we constrict the roi size apriltags shouldn't run out of ram.
        # But, if it does we handle it...
        try:
            # sub_img = img.crop(roi=roi,copy=True)
            # bin_thresh = sub_img.get_histogram().get_threshold()
            # sub_img.binary([(bin_thresh.value(),255)])

            # img.draw_image(sub_img, roi[0], roi[1])
            tag_list.extend(img.find_apriltags(roi=roi, families=tag_families))
        except (
            MemoryError
        ):  # Don't catch all exceptions otherwise you can't stop the script.
            pass


    # img.crop(x_scale = .25, y_scale = .25, hint = image.BILINEAR)
    # tag_list.extend(img.find_apriltags(families=tag_families))


    # img.draw_image(blob_img)
    for b in box_list:
        img.draw_rectangle(b, color=150)
    for tag in tag_list:
        img.draw_rectangle(tag.rect)
        img.draw_cross(tag.cx, tag.cy)
        for c in tag.corners:
            img.draw_circle(c[0], c[1], 5)
        print("Tag:", tag.cx, tag.cy, tag.rotation, tag.id)

    # sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, False)
    # sensor.set_windowing(160,120)
    # img = sensor.snapshot()



