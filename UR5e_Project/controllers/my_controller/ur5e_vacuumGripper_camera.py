import sys
webot_path = '/Users/sina.fazel/Downloads/ai-in-robotics-course-project-AUT2024-main/UR5e_Project/worlds'
sys.path.appensd(webot_path)

import numpy as np
from controller import Robot, VacuumGripper ,  Motor , Camera, RangeFinder

# create the Robot instance.
robot = Robot()

  
  
TIME_STEP = 64

camera = Camera('camera')
range_finder = RangeFinder('range-finder')
set_arm_pos(self , [0,0,0,0,0])

range_finder.enable(TIME_STEP)
camera.enable(TIME_STEP)

## grab RGB frame and convert to numpy ndarray
def get_rgb_frame() -> np.ndarray :
    image_array = camera.getImageArray()
    np_image = np.array(image_array, dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 3))
    return np_image

## grab Depth frame and convert to numpy ndarray
def get_depth_frame() -> np.ndarray :
    image_array = range_finder.getImageArray()
    np_image = np.array(image_array, dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 3))
    return np_image


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
    

## robot instance
ur5 = UR5e() 

a = [0,0,0 ,0,0,0] 
ur5.set_arm_pos(a)
ur5.set_gripper_pos(state = 'on')

robot.step(10*TIME_STEP)

img = get_rgb_frame()    
depth = get_depth_frame()


## Just an Example
t_ms = 0
while robot.step(TIME_STEP) != -1:
    t_ms += TIME_STEP
    t = t_ms / 1000.0 
    
    q1 = np.mod(t , 2.0) - 1.0
    a = [0,q1,0 ,0,0,0] 
    ur5.set_arm_pos(a)
    print(ur5.get_arm_pos())
    
    img = get_rgb_frame()    
    depth = get_depth_frame()


