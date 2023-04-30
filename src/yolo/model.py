import torch
import cv2
from ..camera import Camera

class Model:
    def __init__(self):
        panel_detector_path = 'src/Model/weights/yolov5s.pt'
        torch.hub._validate_not_a_forked_repo = lambda a, b, c: True
        detector = torch.hub.load('ultralytics/yolov5', 'custom', path=panel_detector_path, force_reload=True)
        
        self.detector = detector
        self.result = []
        self.labels = []
        self.coord = []
        self.closest_coord = []
        self.box_center = []
        self.panel_num = []

    # image preprocessing for yoloV5
    def preprocess(self, image):
        # print(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (640,640))
        return image

    # labels and coordinates of bounding box
    def split_result(self):
        self.labels = self.result.xyxyn[0][:, -1].to('cpu').numpy()
        self.coord = self.result.xyxyn[0][:, :-1].to('cpu').numpy()

    # calculate coordinates in pixel values
    def find_coordinates(self, image, coord):
        x_shape, y_shape = image.shape[1], image.shape[0]
        x1 = int(coord[0] * x_shape)
        y1 = int(coord[1] * y_shape)
        x2 = int(coord[2] * x_shape)
        y2 = int(coord[3] * y_shape)

        coordinates = [x1, y1, x2, y2]
        return coordinates 

if __name__ == '__main__':
    pass