#!/usr/bin/env python3

# Author: Amirmohammad Radmehr

# *********     Face orientation function      *********
# This function extracts the 468 points mesh of the face  located in front of the camera 
# Using basic algebraic analysis, angles along X, Y and Z axis are extracted 
# Prameters calculated in this script are:
#               angle along X
#               angle along Y
#               angle along Z
#               frames per second
#               face mesh and connections

# Z axis
#  __
# |\
#   \
#    \
#     \
#      \-------------------->  X axis
#      | 
#      | 
#      | 
#      | 
#      | 
#      | 
#      | 
#      | 
#      \/ 
#      Y axis

from helper import *

import time
import serial
import rospy
from std_msgs.msg import String

# ser = serial.Serial('/dev/ttyACM0',
#                      baudrate=9600,
#                      bytesize=serial.EIGHTBITS,
#                      parity=serial.PARITY_NONE,
#                      stopbits=serial.STOPBITS_ONE,
#                      timeout=1,
#                      xonxoff=0,
#                      rtscts=0
#                      )


# ser.setDTR(False)
# ser.flushInput()
# ser.setDTR(True)

# while True:
#     if ser.inWaiting():
#         dick = str(ser.readline().decode()).strip()
#         print(dick)
#         if dick == 'eye':
#             break


mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils 
drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
LANDMARKS = []

# read the webcam
cap = cv2.VideoCapture(0)

width  = int(cap.get(3)) # frame width
height = int(cap.get(4)) # frame height
image_fake = np.full((int(height), int(width), 3),255, dtype = np.uint8) # plane white surface

face_mesh = mp_face_mesh.FaceMesh(static_image_mode=True,
                                max_num_faces=1,
                                min_detection_confidence=0.5, min_tracking_confidence=0.1)


rospy.init_node('face_angles', anonymous=True)
pub = rospy.Publisher('angles', String, queue_size=10)

def facepose():
    t1 = 0
    while True:

        # capturing a frame
        _, frame = cap.read()
        frame = cv2.flip(frame, 1)
        
        # processing the frame
        results = face_mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if results.multi_face_landmarks:

            annotated_image = image_fake.copy()
            LANDMARKS = []
            # Draw face landmarks of each face.
            for face_landmarks in results.multi_face_landmarks:
                mp_drawing.draw_landmarks(
                    image=annotated_image,
                    landmark_list=face_landmarks,
                    connections=mp_face_mesh.FACE_CONNECTIONS,
                    landmark_drawing_spec=drawing_spec,
                    connection_drawing_spec=drawing_spec)
                
                # saving lanmarks in another list
                LANDMARKS.append(face_landmarks.landmark)

                points = extract_landmarks(LANDMARKS, width, height) # lanmark coordinates
                points_normal = extract_landmarks(LANDMARKS, width, height, normal=True) # landmark points normalized
            
            # centre = np.mean(points,axis=0).astype(int)
            centre = points[1]
            servo1 = int((centre[0]/width)*180)
            servo2 = int((1 - centre[1]/height)*180)
            cv2.circle(annotated_image, (centre[0],centre[1]), 5, RED_COLOR, -1)
            # print(centre)
            # print((str(servo1)+','+str(servo2)+'\n'))
            # print(servo1,servo2)
            # ser.write((str(servo1)+','+str(servo2)+'\n').encode())
            # ser.write(b'70,70\n')

            # cnt = 20
            # ser.write(str.encode("hello"))
            draw_axes(annotated_image)
            # calculating the rotation along Z axis
            z_angle = Z_angle(points, annotated_image)

            # calculating the rotation along Y axis
            y_angle = Y_angle(points_normal,points, annotated_image)
            
            # calculating the rotation along X axis
            x_angle = X_angle(points_normal, points, annotated_image)
            
            pub.publish(str(x_angle)+','+str(y_angle)+','+str(z_angle))
            
            # calculating FPS
            t2 = time.time()
            fps = 1/(t2-t1)
            t1=t2
            cv2.putText(annotated_image,f"FPS:{fps: .0f}", (10,20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, GREEN_COLOR,2)

            
            cv2.imshow('frame', annotated_image)


        else:
            cv2.imshow('frame', image_fake)

        # Exit when escape is pressed
        if cv2.waitKey(delay=1) == 27:
            # When everything done, release the video capture and video write objects
            cap.release()

            # Close all windows
            cv2.destroyAllWindows()
            break

facepose()
# ser.close()
