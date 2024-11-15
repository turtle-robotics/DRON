#!/usr/bin/env python3
from picamera2 import Picamera2

picam2a = Picamera2(0)
picam2b = Picamera2(1)

config = picam2a.create_still_configuration()
picam2a.configure(config)

config = picam2b.create_still_configuration()
picam2b.configure(config)

picam2a.start()
picam2b.start()

picam2a.capture_file("stereoLeft.jpg")
picam2b.capture_file("stereoRight.jpg")

picam2a.stop()
picam2b.stop()
