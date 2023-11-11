from threading import Thread
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import cv2
from module.Static import STATIC_STREAM
import time
from PIL import Image
from PIL.ImageQt import ImageQt

class ShowCamera(QThread):
    def __init__(self, label):
        super(ShowCamera, self).__init__()
        self.label = label

    def run(self):
        while True:
            if not len(STATIC_STREAM.Url) or not STATIC_STREAM.IsStream:
                time.sleep(3)
                continue
            cap = cv2.VideoCapture(STATIC_STREAM.Url)
            if not cap.isOpened():
                time.sleep(3)
                continue
            STATIC_STREAM.Restart = False
            while True:
                isFrame, frame = cap.read()
                if not isFrame:
                    break
                frame = cv2.resize(frame, STATIC_STREAM.Resolution)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = Image.fromarray(frame)
                self.label.setPixmap(QPixmap.fromImage(ImageQt(frame)))

                if not STATIC_STREAM.IsStream or STATIC_STREAM.Restart:
                    break
            cap.release()