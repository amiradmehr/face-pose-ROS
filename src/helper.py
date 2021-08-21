import mediapipe as mp
import numpy as np
import cv2
import math
from mediapipe.python.solutions.drawing_utils import GREEN_COLOR, RED_COLOR, BLUE_COLOR

''' VARIABLES '''

# this dictionary contains mesh indexes for all 468 points and their corresponding parts of a face
MESH = {
  'silhouette': [
    10,  338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288,
    397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136,
    172, 58,  132, 93,  234, 127, 162, 21,  54,  103, 67,  109],

  'lipsUpperOuter': [61, 185, 40, 39, 37, 0, 267, 269, 270, 409, 291],
  'lipsLowerOuter': [146, 91, 181, 84, 17, 314, 405, 321, 375, 291],
  'lipsUpperInner': [78, 191, 80, 81, 82, 13, 312, 311, 310, 415, 308],
  'lipsLowerInner': [78, 95, 88, 178, 87, 14, 317, 402, 318, 324, 308],

  'rightEyeUpper0': [246, 161, 160, 159, 158, 157, 173],
  'rightEyeLower0': [33, 7, 163, 144, 145, 153, 154, 155, 133],
  'rightEyeUpper1': [247, 30, 29, 27, 28, 56, 190],
  'rightEyeLower1': [130, 25, 110, 24, 23, 22, 26, 112, 243],
  'rightEyeUpper2': [113, 225, 224, 223, 222, 221, 189],
  'rightEyeLower2': [226, 31, 228, 229, 230, 231, 232, 233, 244],
  'rightEyeLower3': [143, 111, 117, 118, 119, 120, 121, 128, 245],

  'rightEyebrowUpper': [156, 70, 63, 105, 66, 107, 55, 193],
  'rightEyebrowLower': [35, 124, 46, 53, 52, 65],

  'leftEyeUpper0': [466, 388, 387, 386, 385, 384, 398],
  'leftEyeLower0': [263, 249, 390, 373, 374, 380, 381, 382, 362],
  'leftEyeUpper1': [467, 260, 259, 257, 258, 286, 414],
  'leftEyeLower1': [359, 255, 339, 254, 253, 252, 256, 341, 463],
  'leftEyeUpper2': [342, 445, 444, 443, 442, 441, 413],
  'leftEyeLower2': [446, 261, 448, 449, 450, 451, 452, 453, 464],
  'leftEyeLower3': [372, 340, 346, 347, 348, 349, 350, 357, 465],

  'leftEyebrowUpper': [383, 300, 293, 334, 296, 336, 285, 417],
  'leftEyebrowLower': [265, 353, 276, 283, 282, 295],

  'midwayBetweenEyes': [168],

  'noseTip': [1],
  'noseBottom': [2],
  'noseRightCorner': [98],
  'noseLeftCorner': [327],

  'rightCheek': [205],
  'leftCheek': [425]
  }

FULL_MESH = sum(list(MESH.values()), [])
LEFT_EYE_MESH = MESH['leftEyeUpper0'] + MESH['leftEyeLower0'] + MESH['leftEyeUpper1'] + MESH['leftEyeLower1']
RIGHT_EYE_MESH = MESH['rightEyeUpper0'] + MESH['rightEyeLower0'] + MESH['rightEyeUpper1'] + MESH['rightEyeLower1']
MAX_DEG = 10
''' FUNCTIONS '''

def extract_landmarks(landmarks, width, height, normal = False):
  # inputs landmarks and image rows and columns
  # outputs normalized mesh (p_normal): shape = (468,3) column 0: x, column 1: y, column 2: z
  # outputs mesh coordinates (points): shape = (468,2) column 0: x, column 1: y
  p_normal = []
  for lm in landmarks[0]:
      p_normal.append([lm.x, lm.y, lm.z])
  p_normal = np.array(p_normal)
  if normal:
    return np.array(p_normal)

  points = []
  for i,j in zip(p_normal[:,0], p_normal[:,1]):
      p = mp.solutions.drawing_utils._normalized_to_pixel_coordinates(
      normalized_x=i, normalized_y=j,
      image_width=width, image_height=height)
      if p == None:
        points.append((320,240))
      else:
        points.append(p)
  points = np.array(points)
  return points

def normal_to_pixel(p_normal,width, height):
  points = []
  for i,j in zip(p_normal[:,0], p_normal[:,1]):
      p = mp.solutions.drawing_utils._normalized_to_pixel_coordinates(
      normalized_x=i, normalized_y=j,
      image_width=width, image_height=height)
      if p == None:
        points.append((-1,-1))
      else:
        points.append(p)
  points = np.array(points)
  return points

def angle(start, end):
  m = (start-end)/2
  angle_z = math.atan(m[1]/m[0])
  return math.degrees(angle_z)

def two_vector_angle(vec1, vec2):
  unit_vec1 = vec1 / np.linalg.norm(vec1)
  unit_vec2 = vec2 / np.linalg.norm(vec2)
  dot_product = np.dot(unit_vec1, unit_vec2)
  angle = np.arccos(dot_product)

  return math.degrees(angle)

def X_angle(point_normal, coordinates, image):
    left_eye_normal = point_normal[LEFT_EYE_MESH]
    right_eye_normal = point_normal[RIGHT_EYE_MESH]
    between_eyes = np.mean((left_eye_normal+right_eye_normal)/2, axis=0)
    nose_bottom = point_normal[MESH['noseBottom']][0][[1,2]]
    left_eye_points = coordinates[LEFT_EYE_MESH]
    left_eye_center = np.mean(left_eye_points, axis=0).astype(int)
    right_eye_points = coordinates[RIGHT_EYE_MESH]
    right_eye_center = np.mean(right_eye_points, axis=0).astype(int)
    deg_x = (angle(between_eyes[[1,2]], nose_bottom) + 20)
    if deg_x > MAX_DEG:
      deg_x = MAX_DEG
    elif deg_x<-MAX_DEG:
      deg_x = -MAX_DEG

    cv2.arrowedLine(image, 
                    tuple(coordinates[MESH['noseBottom']][0]),
                    tuple(((right_eye_center+left_eye_center)/2).astype(int)),
                            RED_COLOR, 1)

    cv2.putText(image, "X vec", tuple(coordinates[MESH['noseBottom']][0]),cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0,0,0), 1, cv2.LINE_AA)
    cv2.putText(image, f'Angle along X: {deg_x: .2f}', 
                (50,90), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, RED_COLOR, 1, cv2.LINE_AA)

    return deg_x

def Y_angle(point_normal, coordinates, image):
    right_jaw = point_normal[MESH['silhouette']][27][[0,2]]
    left_jaw = point_normal[MESH['silhouette']][9][[0,2]]
    deg_y = angle(right_jaw, left_jaw)
    if deg_y > MAX_DEG:
      deg_y = MAX_DEG
    elif deg_y<-MAX_DEG:
      deg_y = -MAX_DEG

    vec1 = coordinates[MESH['silhouette']][27]
    vec2 = coordinates[MESH['silhouette']][9]
    cv2.arrowedLine(image, tuple(vec1), tuple(vec2), RED_COLOR, 1)

    between_jaws = (vec1+vec2)/2
    between_jaws = between_jaws.astype(int)
    cv2.putText(image, "Y vec", tuple(between_jaws),cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0,0,0), 1, cv2.LINE_AA)
    cv2.putText(image, f'Angle along Y: {deg_y: .2f}', 
                (50,70), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, RED_COLOR, 1, cv2.LINE_AA)

    return deg_y

def Z_angle(point, image):
    left_eye_points = point[LEFT_EYE_MESH]
    left_eye_center = np.mean(left_eye_points, axis=0).astype(int)
    right_eye_points = point[RIGHT_EYE_MESH]
    right_eye_center = np.mean(right_eye_points, axis=0).astype(int)

    deg_z = angle(right_eye_center, left_eye_center)
    if deg_z > MAX_DEG:
      deg_z = MAX_DEG
    elif deg_z<-MAX_DEG:
      deg_z = -MAX_DEG
    cv2.arrowedLine(image, tuple(left_eye_center), tuple(right_eye_center), RED_COLOR, 1)
    
    between_eyes = (right_eye_center + left_eye_center)/2
    between_eyes = between_eyes.astype(int)
    cv2.putText(image, "Z vec", tuple(between_eyes),cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0,0,0), 1, cv2.LINE_AA)
    cv2.putText(image, f'Angle along Z: {deg_z: .2f}', 
                (50,50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, RED_COLOR, 1, cv2.LINE_AA)

    return deg_z

def draw_axes(image):
  image_rows, image_cols, _ = image.shape
  origin = (1,1)
  cv2.arrowedLine(image, origin, (1,image_rows), RED_COLOR, 2)
  cv2.arrowedLine(image, origin, (image_cols,1), GREEN_COLOR, 2)
  cv2.arrowedLine(image, origin, (40,20), BLUE_COLOR, 2)

def PolyArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

def blink_angle(blinkvalue):
  val = 180 * (blinkvalue-.3)/(3-.3)
  if val<0:
    return 0
  elif val>180:
    return 180
  else:
    return val