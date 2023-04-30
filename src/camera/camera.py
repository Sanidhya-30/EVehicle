## Camera Class / Image Capturing / OpenCV processing

import cv2
import numpy as np

class Camera:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
         # check/change parameter before testing

    def capture(self):
        _, src = self.cap.read()                  
        return src
    
if __name__== "__main__":
    pass