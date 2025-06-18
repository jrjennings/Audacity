# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Multi Color Blob Tracking Example
#
# This example shows off multi color blob tracking using the OpenMV Cam.

import sensor
import time
import math

from pyb import UART

uart = UART(3, 19200, timeout_char=200)

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green things. You may wish to tune them...
thresholds = [
    (26, 68, 36, 80, 0, 83)#,   generic_red_thresholds
    #(30, 100, -64, -8, -32, 32),  # generic_green_thresholds
    #(0, 15, 0, 40, -80, -20),
]  # generic_blue_thresholds
# You may pass up to 16 thresholds above. However, it's not really possible to segment any
# scene 3with 16 thresholds before color thresholds start to overlap heavily.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. Don't set "merge=True" because that will merge blobs which we don't want here.


def is_triangle(blob):
    # Calculate the ratio of pixels to bounding rectangle area
    rect_area = blob.w() * blob.h()
    if rect_area > 0:  # Avoid division by zero
        pixel_ratio = blob.pixels() / rect_area
        # Check if the ratio is close to 0.5 (with some tolerance)
        # You'll need to experiment to find the optimal tolerance value
        if abs(pixel_ratio - 0.5) < 0.01:  # Adjust tolerance as needed
            return True
        else:
            return False


while True:
    clock.tick()
    img = sensor.snapshot()
    blob_list = img.find_blobs(thresholds, pixels_threshold=200, area_threshold=200, feature_filter=is_triangle)
    blob_count = len(blob_list)


    for blob in blob_list:
        # These values depend on the blob not being circular - otherwise they will be shaky.
        #if blob.elongation() > 0.5:
            #img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            #img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            #img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
        # These values are stable all the time.
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints(
            [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
        )
    #myData blob size, blob center, ratio of width to height
    myData = blob.pixels(), blob.cx()-160, round (blob.h()/ blob.w(), 1)
    if (blob_count == 1):
        stringData = str(myData)+ "\r\n"
    else:
        stringData = str("(0, 0, 0)")+ "\r\n"
    uart.write(stringData)
    time.sleep_ms(100)
