
# !pip install ultralytics
import sys
import numpy as np
import time

from math import sqrt,pi,cos,sin, atan2
from ultralytics import YOLO


from controller import Robot, VacuumGripper ,  Motor , Camera, RangeFinder

# create the Robot instance.
robot = Robot()



TIME_STEP = 64

camera = Camera('camera')
range_finder = RangeFinder('range-finder')

range_finder.enable(TIME_STEP)
camera.enable(TIME_STEP)

## grab RGB frame and convert to numpy ndarray
def get_rgb_frame() -> np.ndarray :
    image_array = camera.getImageArray()
    np_image = np.array(image_array, dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 3))
    return np_image

## grab Depth frame and convert to numpy ndarray
# def get_depth_frame() -> np.ndarray :
    # image_array = range_finder.getRangeImageArray()
    # np_image = np.array(image_array, dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 3))
    # return np_image


class UR5e:
    def __init__(self , name="my_robot"):
        # get the motor devices
        m1 = robot.getDevice('shoulder_lift_joint')
        m2 = robot.getDevice('shoulder_pan_joint')
        m3 = robot.getDevice('elbow_joint')

        m4 = robot.getDevice('wrist_1_joint')
        m5 = robot.getDevice('wrist_2_joint')
        m6 = robot.getDevice('wrist_3_joint')

        self.vac = robot.getDevice('vacuum gripper')
        self.vac.enablePresence(1)

        self.motors_list = [m1,m2,m3,m4,m5,m6]

        self.gps = robot.getDevice('gps')
        self.gps.enable(1)

        sampling_period = 1

        for m in self.motors_list:
            m.getPositionSensor().enable(sampling_period)
            m.enableForceFeedback(sampling_period)
            m.enableTorqueFeedback(sampling_period)


    def set_arm_torques(self , torques):
        for i,motor in enumerate(self.motors_list):
            motor.setTorque(torques[i])

    def set_gripper_pos(self, state = 'on'):
        ''' state : set vacuum gripper "on" or "off" for vacuum activation'''
        if state == 'on' or state == 'On' or state == 'ON':
            self.vac.turnOn()
        else:
            self.vac.turnOff()

    def set_arm_pos(self , pos):
        for i,motor in enumerate(self.motors_list):
            motor.setPosition(pos[i])

    def get_arm_pos(self):
        p = [m.getPositionSensor().getValue() for m in self.motors_list]
        return p

    def get_gripper_pos(self):
        p = [m.getPositionSensor().getValue() for m in self.gripper_list]
        return p

    def get_EE_position(self):
        return self.gps.value
       
   
#model = YOLO("/Users/sina.fazel/Desktop/AUT/AI in Robotics/ai-in-robotics-course-project-AUT2024-main/UR5e_Projectbest.pt")


## robot instance
ur5 = UR5e()


# x = 600
# y = 20
# w = 10
# h = 4

# x_p = x + w/2
# y_p = y + h/2

# a = (x_p / 640) - 1/2
# b = (y_p / 640) - 1/2

# x = -(b*0.577) - 0.45
# y = a*0.778



x = 0.4
y = 0.2
z = 0.9

t_ms = 0

# t_ms += TIME_STEP
# t = t_ms / 1000.0
     
     
# a1 = [-1.94,0.23,-0.9 ,1.27,-1.6,-1.33]

# ur5.set_arm_pos(a1)

# time.sleep(4)
     
# a2 = [ 3.30327508, -0.70239683,  1.73138405,  0.5418091,   1.57079633,  1.73247876]
# ur5.set_arm_pos(a2)


img = get_rgb_frame()  
# depth = get_depth_frame()

#print(Q)

## Just an Example

def delay(ms):
        initTime = robot.getTime()      # Store starting time (in seconds)
        while robot.step(TIME_STEP ) != -1:
            if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
                break


# while robot.step(TIME_STEP) != -1:

     # t_ms += TIME_STEP
     # t = t_ms / 1000.0
     # ur5.set_gripper_pos(state = 'off')
# a1 = [-1.94,0.23,-0.9 ,1.27,-1.6,-1.33]
# ur5.set_arm_pos(a1)
# delay(2000)
     
# Rbox = [ -0.9,-0.5,1.2,-1.57,-1.57,0]
# ur5.set_arm_pos(Rbox)

# delay(2000)    
# Gbox = [ -0.9,0.1,1.2,-1.57,-1.57,0]
# ur5.set_arm_pos(Gbox)

# delay(2000) 
# Bbox = [ -1.1,1.3,1.2,-1.57,-1.57,0]  
# ur5.set_arm_pos(Bbox)
# delay(1000)  
# Bbox = [-0.5,1.3,1.6,-2.7,-1.57,0]
# ur5.set_arm_pos(Bbox)

a1 = [0,3.14,0,0,0,0]
ur5.set_arm_pos(a1)
delay(2000)

# a1 = [-1.57,0,0,0,0,0]
# ur5.set_arm_pos(a1)
# delay(3000)

# a1 = [0,0,1.57,0,0,0]
# ur5.set_arm_pos(a1)
# delay(3000)


    # q1 = np.mod(t , 2.0) - 1.0
    # a = [0,q1,0 ,0,0,0]
    # ur5.set_arm_pos(a)
    # print(ur5.get_arm_pos())

     # img = get_rgb_frame()  
    # #depth = get_depth_frame()