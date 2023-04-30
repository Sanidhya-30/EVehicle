## Lidar Data capturing / tuning / filtering / outlier removal

#Class for Ultrasonic as for proximity sensors

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Lidar:
    def __init__(self):
        print('Created LiDAR')
        self.drive_ok = False
        self.area_completed = False
        # GPIO.setup(self.TRIGGER, GPIO.OUT)
        # GPIO.setup(self.ECHO, GPIO.IN)

    def get_pointcloud(self):
        return

if __name__ == '__main__':
    pass