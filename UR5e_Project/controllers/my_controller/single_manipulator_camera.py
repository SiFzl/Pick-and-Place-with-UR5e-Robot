import sys
webot_path = 'D:\Program Files\Webots\lib\controller\python'
sys.path.append(webot_path)

import numpy as np
from controller import Robot, Motor , Camera, RangeFinder

# create the Robot instance.
robot = Robot()
camera = Camera('camera')
range_finder = RangeFinder('range-finder')
range_finder.enable(10)
camera.enable(10)
TIME_STEP = 64


class UR5e:
    def __init__(self , name="my_robot"):
        # get the motor devices
        m1 = robot.getDevice('shoulder_lift_joint')
        m2 = robot.getDevice('shoulder_pan_joint')
        m3 = robot.getDevice('elbow_joint')

        m4 = robot.getDevice('wrist_1_joint')
        m5 = robot.getDevice('wrist_2_joint')
        m6 = robot.getDevice('wrist_3_joint')
        
        self.griper_finger_l = robot.getDevice('ROBOTIQ 2F-140 Gripper::left finger joint')
        self.griper_finger_r = robot.getDevice('ROBOTIQ 2F-140 Gripper::right finger joint')
        self.motors_list = [m1,m2,m3,m4,m5,m6]
        self.gripper_list = [self.griper_finger_r , self.griper_finger_l]
        
        self.gps = robot.getDevice('gps')
        self.gps.enable(1)

        sampling_period = 1
        self.griper_finger_r.getPositionSensor().enable(sampling_period)
        self.griper_finger_l.getPositionSensor().enable(sampling_period)
        
        for m in self.motors_list:
            m.getPositionSensor().enable(sampling_period)
            m.enableForceFeedback(sampling_period)
            m.enableTorqueFeedback(sampling_period)
           
            
    def set_arm_torques(self , torques):
        for i,motor in enumerate(self.motors_list):
            motor.setTorque(torques[i])
    
    def set_gripper_pos(self, pos):
        ''' 0.8 > pos > 0 '''
        self.griper_finger_l.setPosition(pos)
        self.griper_finger_r.setPosition(pos)
        
        
    def set_gripper_force(self , force):
        ''' -10 N > force > 10 N'''
        self.griper_finger_l.setForce(force)
        self.griper_finger_r.setForce(force)
        
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
    
  

ur5 = UR5e() 
a = [0,0,0 ,0,0,-1.570] 
ur5.set_arm_pos(a)
ur5.set_gripper_pos(0)
robot.step(100*TIME_STEP)

b = [0 , -1.57 , .8 ,  1.57, -1.57, -1.0]
ur5.set_arm_pos(b)
robot.step(100*TIME_STEP)


img = camera.getImageArray()
depth = range_finder.getRangeImageArray()

## TODO:
# open gripper
# go to box_init_pos = [-0.6 , 0.2 , 0.8]
# close gripper
# box_final_pos = [0.65 , -0.3 , 0.6]
# open gripper

t_ms = 0
while robot.step(TIME_STEP) != -1:
    t_ms += TIME_STEP
    t = t_ms / 1000.0 
    
    q1 = np.mod(t , 2.0) - 1.0
    a = [0,q1,0 ,0,0,0] 
    ur5.set_arm_pos(a)
    print(ur5.get_arm_pos())
    
