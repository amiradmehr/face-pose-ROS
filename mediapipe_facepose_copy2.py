import mediapipe as mp
from cv2 import cv2
from mediapipe.python.solutions.drawing_utils import GREEN_COLOR, RED_COLOR
import numpy as np
from helper import *
import time
from copy import deepcopy

mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils 
drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
LANDMARKS = []

# read the webcam
cap = cv2.VideoCapture(0)
t1 = 0
width  = int(cap.get(3))
height = int(cap.get(4))
image_fake = np.full((int(height), int(width), 3),255, dtype = np.uint8)


face_mesh = mp_face_mesh.FaceMesh(static_image_mode=True,
                                  max_num_faces=1,
                                  min_detection_confidence=0.5, min_tracking_confidence=0.1)


