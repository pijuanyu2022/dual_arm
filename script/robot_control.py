#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range, JointState
import tf2_ros
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
import time
import numpy as np
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
import math
from dual_arm.msg import Movement


class RobotControl(object):
    def __init__(self, robot_type='UR5'):
        self.robot_type = robot_type
        if self.robot_type == 'UR5':
            rospy.init_node('UR5controller', anonymous=True)
            
        print('INIT NODE')
        
        self.initialize_subscribers()
        self.sim_time = 0.0
        
        self.robot_pose = Pose()
        self.Right_RG2_front1_sensor_val = Float32()
        self.Right_RG2_front2_sensor_val = Float32()
        self.Right_RG2_touch_sensor_val = Float32()
        self.Right_Conveyor_belt_sensor_val = Float32()
        self.Movement_val = Int32()

        self.Left_RG2_front1_sensor_val = Float32()
        self.Left_RG2_front2_sensor_val = Float32()
        self.Left_RG2_touch_sensor_val = Float32()
        self.Left_Conveyor_belt_sensor_val = Float32()

        self.SENSOR_TYPES = ['Right_RG2_front1', 'Right_RG2_front2','Right_RG2_touch', 'Right_Conveyor_belt', \
            'Left_RG2_front1', 'Left_RG2_front2','Left_RG2_touch', 'Left_Conveyor_belt', 'Person_val', 'Movement_val']

    def degToRad(self, deg):
        return deg*math.pi/180.0
    
    def initialize_subscribers(self):
        rospy.Subscriber('/simTime', Float32, self.simTime_cb)
        rospy.Subscriber('/tf', TFMessage, self.tf_cb)

        rospy.Subscriber('/Right_RG2_front1SensorDistance', Range, self.distance_Right_RG2_front1_cb)
        rospy.Subscriber('/Right_RG2_front2SensorDistance', Range, self.distance_Right_RG2_front2_cb)
        rospy.Subscriber('/Right_RG2_touchSensorDistance',  Range, self.distance_Right_RG2_touch_cb)
        rospy.Subscriber('/Right_Conveyor_beltSensorDistance', Range, self.distance_Right_Conveyor_belt_cb)

        rospy.Subscriber('/Left_RG2_front1SensorDistance', Range, self.distance_Left_RG2_front1_cb)
        rospy.Subscriber('/Left_RG2_front2SensorDistance', Range, self.distance_Left_RG2_front2_cb)
        rospy.Subscriber('/Left_RG2_touchSensorDistance',  Range, self.distance_Left_RG2_touch_cb)
        rospy.Subscriber('/Left_Conveyor_beltSensorDistance', Range, self.distance_Left_Conveyor_belt_cb)
        rospy.Subscriber("/move_info", Movement, self.movementInfoCallback)
    
    # subscriber callbacks
    def simTime_cb(self, msg):
        self.sim_time = msg.data
    
    def tf_cb(self, msg):
        self.robot_pose.position.x = msg.transforms[0].transform.translation.x
        self.robot_pose.position.y = msg.transforms[0].transform.translation.y
        self.robot_pose.position.z = msg.transforms[0].transform.translation.z
        self.robot_pose.orientation.x = msg.transforms[0].transform.rotation.x
        self.robot_pose.orientation.y = msg.transforms[0].transform.rotation.y
        self.robot_pose.orientation.z = msg.transforms[0].transform.rotation.z
        self.robot_pose.orientation.w = msg.transforms[0].transform.rotation.w

    #Sensor callbacks Same for both robots. 
    def movementInfoCallback (self,msg):
        self.Movement_val.data = msg.step  

    def distance_Right_RG2_front1_cb(self, msg):
        self.Right_RG2_front1_sensor_val.data = msg.range

    def distance_Right_RG2_front2_cb(self, msg):
        self.Right_RG2_front2_sensor_val.data = msg.range
    
    def distance_Right_RG2_touch_cb(self, msg):
        self.Right_RG2_touch_sensor_val.data = msg.range    

    def distance_Right_Conveyor_belt_cb(self, msg):
        self.Right_Conveyor_belt_sensor_val.data = msg.range
        

    def distance_Left_RG2_front1_cb(self, msg):
        self.Left_RG2_front1_sensor_val.data = msg.range

    def distance_Left_RG2_front2_cb(self, msg):
        self.Left_RG2_front2_sensor_val.data = msg.range
    
    def distance_Left_RG2_touch_cb(self, msg):
        self.Left_RG2_touch_sensor_val.data = msg.range    

    def distance_Left_Conveyor_belt_cb(self, msg):
        self.Left_Conveyor_belt_sensor_val.data = msg.range

    def joint_states_cb(self, msg):
        self.joint_states = msg

    
    #getters 
    def get_coppeliasim_Value(self, sensor_type='Right_RG2_front1'):
        assert sensor_type in self.SENSOR_TYPES
        if sensor_type=='Right_RG2_front1':
            return self.Right_RG2_front1_sensor_val.data
        elif sensor_type=='Right_RG2_front2':
            return self.Right_RG2_front2_sensor_val.data
        elif sensor_type == 'Right_RG2_touch':
            return self.Right_RG2_touch_sensor_val.data
        elif sensor_type == 'Right_Conveyor_belt':
            return self.Right_Conveyor_belt_sensor_val.data
        elif sensor_type=='Left_RG2_front1':
            return self.Left_RG2_front1_sensor_val.data
        elif sensor_type=='Left_RG2_front2':
            return self.Left_RG2_front2_sensor_val.data
        elif sensor_type == 'Left_RG2_touch':
            return self.Left_RG2_touch_sensor_val.data
        elif sensor_type == 'Left_Conveyor_belt':
            return self.Left_Conveyor_belt_sensor_val.data
        elif sensor_type == 'Movement_val':
            return self.Movement_val.data

    def getRobotWorldLocation(self):
        return self.robot_pose.position, self.robot_pose.orientation
    
    def getCurrentSimTime(self):
        return self.sim_time
    

    