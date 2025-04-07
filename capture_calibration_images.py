import sensor
import time

# Initialize the camera sensor
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_whitebal(False)

image_counter = 0  # To create unique image filenames
interval = 1000  # milliseconds
next_heartbeat = time.ticks_add(time.ticks_ms(), interval)
while(True):
    img = sensor.snapshot()
    if time.ticks_diff(time.ticks_ms(), next_heartbeat) >= 0:
        # Take a snapshot
        img = sensor.snapshot()
        image_counter += 1
        filename = "image_%04d.jpg" % (image_counter)
        img.save(filename)
        print("Saved:", filename)
        next_heartbeat = time.ticks_add(next_heartbeat, interval)
