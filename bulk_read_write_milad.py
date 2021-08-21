#!/usr/bin/env python3
# -*- coding: utf-8 -*-import math

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific lang250uage governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Bulk Read and Bulk Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 and 2 / Baudnum : 1 (Baudrate : 57600)
#
import os
import math
import numpy as np
from numpy.core.numeric import indices
from agileye import *
# from mediapipe_facepose_copy2 import *
import rospy
from std_msgs.msg import String

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_PRO_GOAL_VELOCITY      = 104
ADDR_PRO_POSITION_P         = 84
ADDR_PRO_POSITION_I         = 82
ADDR_PRO_POSITION_D         = 80
ADDR_PRO_VELOCITY_P         = 78
ADDR_PRO_VELOCITY_I         = 76

# Data Byte Length

LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 2073-5               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2073+5          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 100               # Dynamixel moving status threshold

# desired end effector rotation

x_val = 0
y_val = 0
z_val = 0

def callback(data):
    global x_val,y_val,z_val
    x_val,y_val,z_val = data.data.split(',')
    x_val = float(x_val)
    y_val = float(y_val)
    z_val = float(z_val)
    

rospy.init_node('agile_eye', anonymous=True)






rot = ikp(0,0,0)
n = 1                                         
# dxl1_goal_position= [rot[0]]*n
# dxl2_goal_position= [rot[1]]*n
# dxl3_goal_position= [rot[2]]*n
# for i in range(n):
#     dxl1_goal_position[i] = 2048
index = 0             
# dxl2_goal_position = [x for x in dxl1_goal_position]                                        
# dxl3_goal_position = [x for x in dxl1_goal_position]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupBulkWrite instance
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

# Initialize GroupBulkRead instace for Present Position
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# Enable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)

# postion PID tunning
PID_P = [100, 50, 5]
# PID_P = [70, 50, 15]
PID_P = [200, 50, 15]
PID_V = [100, 100]

# PID_V = [900, 900]
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P, PID_P[0])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P, PID_P[0])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P, PID_P[0])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I, PID_P[1])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I, PID_P[1])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I, PID_P[1])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D, PID_P[2])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D, PID_P[2])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D, PID_P[2])
# velocity PI tunning
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_P, PID_V[0])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_P, PID_V[0])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_P, PID_V[0])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_I, PID_V[1])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_I, PID_V[1])
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_I, PID_V[1])


# Add parameter storage for Dynamixel#1 present position
dxl_addparam_result = groupBulkRead.addParam(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL1_ID)
    quit()

# Add parameter storage for Dynamixel#2 present position
dxl_addparam_result = groupBulkRead.addParam(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL2_ID)
    quit()

# Add parameter storage for Dynamixel#3 present position
dxl_addparam_result = groupBulkRead.addParam(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL3_ID)
    quit()

flag = 0
while 1:
    if (index == 0):
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            # When everything done, release the video capture and video write objects
            # cap.release()

            # # Close all windows
            # cv2.destroyAllWindows()
            break
    
    index = 0
    # _, frame = cap.read()
    # frame = cv2.flip(frame, 1)
    
    # # processing the frame
    # results = face_mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # if results.multi_face_landmarks:

    #     annotated_image = image_fake.copy()
    #     LANDMARKS = []
    #     # Draw face landmarks of each face.
    #     for face_landmarks in results.multi_face_landmarks:
    #         mp_drawing.draw_landmarks(
    #             image=annotated_image,
    #             landmark_list=face_landmarks,
    #             connections=mp_face_mesh.FACE_CONNECTIONS,
    #             landmark_drawing_spec=drawing_spec,
    #             connection_drawing_spec=drawing_spec)
            
    #         # saving lanmarks in another list
    #         LANDMARKS.append(face_landmarks.landmark)

    #         # extracting lanmark coordinates
    #         points = extract_landmarks(LANDMARKS, width, height)
    #         points_normal = extract_landmarks(LANDMARKS, width, height, normal=True)

    #     # calculating the rotation along Z axis
    #     z_angle = Z_angle(points, annotated_image)

    #     # calculating the rotation along Y axis
    #     y_angle = Y_angle(points_normal,points, annotated_image)
        
    #     # calculating the rotation along X axis
    #     x_angle = X_angle(points_normal, points, annotated_image)
    rospy.Subscriber('angles', String, callback)

    rot = ikp(z_val, y_val, x_val)

    dxl1_goal_position = [rot[0]]
    dxl2_goal_position = [rot[1]]
    dxl3_goal_position = [rot[2]]
        # calculating FPS
    #     t2 = time.time()
    #     fps = 1/(t2-t1)
    #     t1=deepcopy(t2)
    #     cv2.putText(annotated_image,f"FPS:{fps: .0f}", (10,20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, GREEN_COLOR,2)


    #     cv2.imshow('frame', annotated_image)
    
    # else:
    #     cv2.imshow('frame', image_fake)

    # Exit when escape is pressed
    # if cv2.waitKey(delay=1) == 27:
    #     break

    # Allocate goal position value into byte array
    param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position[index])),
    DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position[index])),
    DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position[index]))]

    param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position[index])),
    DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position[index])),
    DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position[index]))]

    param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(dxl3_goal_position[index])),
    DXL_HIBYTE(DXL_LOWORD(dxl3_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl3_goal_position[index])),
    DXL_HIBYTE(DXL_HIWORD(dxl3_goal_position[index]))]

    # Add Dynamixel#1 goal position value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position_1)
    # if dxl_addparam_result != True:
    #     print("[ID:%03d] groupBulkWrite addparam failed" % DXL1_ID)
    #     quit()

    # Add Dynamixel#2 goal position value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(DXL2_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position_2)
    # if dxl_addparam_result != True:
    #     print("[ID:%03d] groupBulkWrite addparam failed" % DXL2_ID)
    #     quit()

    # Add Dynamixel#3 goal position value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(DXL3_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position_3)
    # if dxl_addparam_result != True:
    #     print("[ID:%03d] groupBulkWrite addparam failed" % DXL3_ID)
    #     quit()

    # Bulkwrite goal position 
    dxl_comm_result = groupBulkWrite.txPacket()
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()

    while 1:
        # Bulkread present position and LED status
        dxl_comm_result = groupBulkRead.txRxPacket()
        # if dxl_comm_result != COMM_SUCCESS:
        #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupbulkread data of Dynamixel#1 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        # if dxl_getdata_result != True:
        #     print("[ID:%03d] groupBulkRead getdata failed" % DXL1_ID)
        #     quit()

        # Check if groupbulkread data of Dynamixel#2 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        # if dxl_getdata_result != True:
        #     print("[ID:%03d] groupBulkRead getdata failed" % DXL2_ID)
        #     quit()

        # Check if groupbulkread data of Dynamixel#3 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        # if dxl_getdata_result != True:
        #     print("[ID:%03d] groupBulkRead getdata failed" % DXL3_ID)
        #     quit()

        # Get present position value
        dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get present position value
        dxl2_present_position = groupBulkRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get present position value
        dxl3_present_position = groupBulkRead.getData(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl1_goal_position[index], dxl1_present_position, DXL2_ID, dxl2_goal_position[index], dxl2_present_position, DXL3_ID, dxl3_goal_position[index], dxl3_present_position))
       
        if ((abs(dxl1_goal_position[index] - dxl1_present_position) < DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl2_goal_position[index] - dxl2_present_position) < DXL_MOVING_STATUS_THRESHOLD)and (abs(dxl3_goal_position[index] - dxl3_present_position) < DXL_MOVING_STATUS_THRESHOLD)):
            index = 1
            break

    # Change goal position
    # if(flag == 0):
    #     if (index < n-1):
    #         index = index +1
    #     else:
    #         flag = 1
    # else:
    #     if ( index > 0):
    #         index = index -1
    #     else:
    #         flag = 0

# Clear bulkread parameter storage
groupBulkRead.clearParam()

# Disable Dynamixel#1 Torqueparam_goal_position
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
