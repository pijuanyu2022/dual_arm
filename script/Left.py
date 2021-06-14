#!/usr/bin/env python
import rospy
import time
import math
import sys
import copy
import sim
import os
import yaml
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from robot_control import RobotControl
from dual_arm.msg import Movement

# connect to remote API server for the Left robot arm (Right robot arm will be connested with another remote API server)
clientID=sim.simxStart('127.0.0.1',20001,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')

# get the joint handles for six joints
jointhandles={}
for i in range (0,7):
    er,jointhandles[i]=sim.simxGetObjectHandle(clientID,('Left_UR5_joint'+str(i+1)),sim.simx_opmode_blocking)

# when the value is larger than 0, the RG2 gripper will open
# when the value is smaller than 0, the RG2 gripper will close

class UR5Control(RobotControl):

    def __init__(self):
        super(UR5Control, self).__init__(robot_type='UR5')
         
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('UR5controller', anonymous=True)
        time.sleep(2.0)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a `MoveGroupCommander`_ object
        group_name="manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)



        # Instantiate a `PlanningSceneInterface`_ object
        scene = moveit_commander.PlanningSceneInterface()       
         # Misc variables
        self.robot = robot
        self.scene = scene
        self.group = group
        self.add_object()  # add ground

        # publish a step information for the assemble, when the movement value is 5,
        # the right robot arm will go from the assembly position to the end position
        move_info_pub = rospy.Publisher('/move_info', Movement, queue_size=10)

        # main control loop
        while not rospy.is_shutdown():

            # read the sensor value
            V_Left_RG2_front1 = self.get_coppeliasim_Value('Left_RG2_front1')
            V_Left_RG2_front2 = self.get_coppeliasim_Value('Left_RG2_front2')

            # Step 1 : when the front sensors does not detect anything
            # the left arm go to the assemblt position
            if V_Left_RG2_front1 < 0.01 and V_Left_RG2_front2 < 0.01:
                # the first three numebers are the orientation
                # the last three numbers are the position
                pickuppos=[math.pi/2, 0, math.pi/2, 0.3558, 0.3562, 0.208]
                
                #generate the plan
                plan = self.plan_path(pickuppos)

                # go to the position, 0.02 means the gripper will open 
                self.execute_cartesian(plan, 0.02)
            
            # Step 2 : when the object is detected by the front sensor
            # the left arm go to the ready position
            if V_Left_RG2_front1 > 0.01 and V_Left_RG2_front2 > 0.01:

                # close to the object
                A = V_Left_RG2_front1-0.047
                plan = self.cartesian_path(math.pi/2, 0, math.pi/2, 0, A, 0)
                self.execute_cartesian(plan, 0.02)

                # pick up the object, -0.03 means the gripper will close 
                plan = self.cartesian_path(math.pi/2, 0, math.pi/2, 0, 0, 0)
                self.execute_plan(plan, -0.03)

                # wait for the right arm to avoid collision of two arms
                rospy.sleep(10)


                # Assembling the first component

                # go to the assembly position
                p1 = [math.pi/2, math.pi, math.pi/2, -0.1175, -0.542, 0.582]
                plan = self.plan_path(p1)
                self.execute_cartesian(plan, -0.03)
                
                plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, -0.0375, -0.025, 0)
                self.execute_cartesian(plan, -0.03)

                # go down
                plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, 0, 0, -0.05)
                self.execute_cartesian(plan, -0.03)
                # open the gripper 
                plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, 0, 0, 0)
                self.execute_cartesian(plan, 0.01)
                # go up
                plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, 0, 0, 0.08)
                self.execute_cartesian(plan, 0.01)

                # Assembling the next three components
                
                # This is a matrix which contains the assembly position of the next three components
                #x = [[-0.08, -0.588, 0.562],    # [x position, y position, z position] #0.582
                #     [-0.08, -0.524, 0.562],
                #     [-0.155, -0.524, 0.562]]

                x = [[0.0375, -0.030, 0],
                     [0.0375, 0.028, 0],
                     [-0.0375, 0.028, 0]]
                i = 0
                for i in range(0,3):
                    # Go back to the ready position
                    p2=[math.pi/2, 0, math.pi/2, 0.3558, 0.3562, 0.208]
                    plan = self.plan_path(p2)
                    self.execute_plan(plan, 0.02)
                    
                    # Pick up the object
                    V_Left_RG2_front1 = self.get_coppeliasim_Value('Left_RG2_front1')
                    B = V_Left_RG2_front1-0.02
                    plan = self.cartesian_path(math.pi/2, 0, math.pi/2, 0, B, 0)
                    self.execute_cartesian(plan, 0.02)
                    plan = self.cartesian_path(math.pi/2, 0, math.pi/2, 0, 0, 0)
                    self.execute_cartesian(plan, -0.03)
                    plan = self.cartesian_path(math.pi/2, 0, math.pi/2, 0, 0, 0.01)
                    self.execute_cartesian(plan, -0.03)
                    

                    p3 = [math.pi/2, math.pi, math.pi/2, -0.1175, -0.542, 0.582]
                    plan = self.plan_path(p3)
                    self.execute_cartesian(plan, -0.03)

                    # Go to the assembly position
                    #p = [math.pi/2, math.pi, math.pi/2, x[i][0], x[i][1], x[i][2]]
                    plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, x[i][0], x[i][1], x[i][2])
                    self.execute_cartesian(plan, -0.03)
                    # Go down
                    plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, 0, 0, -0.05)
                    self.execute_cartesian(plan, -0.03)
                    # open the gripper
                    plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, 0, 0, 0)
                    self.execute_cartesian(plan, 0.01)
                    # Go up
                    plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, 0, 0, 0.07)
                    self.execute_cartesian(plan, 0.01)
                    i = i+1

                # Go back to the ready position to prepare for more components
                pickuppos=[math.pi/2, 0, math.pi/2, 0.3558, 0.3562, 0.208]
                plan = self.plan_path(pickuppos)
                self.execute_plan(plan, 0.01)

                # publish the movement value is 5
                move_msg = Movement()
                move_msg.step = 5
                move_info_pub.publish(move_msg)

            time.sleep(0.1) # change the sleep time to whatever is the appropriate control rate for simulation
            


    def execute_plan(self, data, p):
        # based on the plan, execute the movement of robot in the coppeliasim 
        traj=data.joint_trajectory.points
        for j in range (1,len(traj)):
            targetpos=traj[j].positions
            sim.simxSetJointTargetPosition(clientID,jointhandles[0],targetpos[0]-math.pi/2,sim.simx_opmode_streaming)  # Left arm joint 1
            sim.simxSetJointTargetPosition(clientID,jointhandles[1],targetpos[1]+math.pi/2,sim.simx_opmode_streaming)  # Left arm joint 2
            sim.simxSetJointTargetPosition(clientID,jointhandles[2],targetpos[2],sim.simx_opmode_streaming)            # Left arm joint 3
            sim.simxSetJointTargetPosition(clientID,jointhandles[3],targetpos[3]+math.pi/2,sim.simx_opmode_streaming)  # Left arm joint 4
            sim.simxSetJointTargetPosition(clientID,jointhandles[4],targetpos[4],sim.simx_opmode_streaming)            # Left arm joint 5
            sim.simxSetJointTargetPosition(clientID,jointhandles[5],targetpos[5],sim.simx_opmode_oneshot_wait)         # Left arm joint 6
            sim.simxSetJointTargetVelocity(clientID,jointhandles[6],p, sim.simx_opmode_streaming)                      # RG2 Gripper
        rospy.sleep(2.)  # stop 2 seconds for stable

    def execute_cartesian(self, data, p):
        # based on the cartesian trajectory, execute the movement of the robot 
        # make sure the joint 6 not change to keep stable
        traj=data.joint_trajectory.points
        for j in range (1,len(traj)):
            targetpos=traj[j].positions
            sim.simxSetJointTargetPosition(clientID,jointhandles[0],targetpos[0]-math.pi/2,sim.simx_opmode_streaming) # Left arm joint 1
            sim.simxSetJointTargetPosition(clientID,jointhandles[1],targetpos[1]+math.pi/2,sim.simx_opmode_streaming) # Left arm joint 2
            sim.simxSetJointTargetPosition(clientID,jointhandles[2],targetpos[2],sim.simx_opmode_streaming)           # Left arm joint 3
            sim.simxSetJointTargetPosition(clientID,jointhandles[3],targetpos[3]+math.pi/2,sim.simx_opmode_streaming) # Left arm joint 4
            sim.simxSetJointTargetPosition(clientID,jointhandles[4],targetpos[4],sim.simx_opmode_streaming)           # Left arm joint 5
            i = len(traj)-1
            sim.simxSetJointTargetPosition(clientID,jointhandles[5],traj[i].positions[5],sim.simx_opmode_oneshot_wait)# Left arm joint 6
            sim.simxSetJointTargetVelocity(clientID,jointhandles[6],p, sim.simx_opmode_streaming)                     # RG2 Gripper
        rospy.sleep(2.)  # stop 2 seconds for stable

    def add_object(self):
        # add a object as the ground
        scene = self.scene 
        scene.remove_world_object("ground")
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.y=0
        box_pose.pose.position.x=0
        box_pose.pose.position.z = -0.5 
        box_name = "ground"
        scene.add_box(box_name, box_pose, size=(10, 10, 1))
    
    def plan_path(self, p):
        # use move_group.plan to get a plan path
        # input the target pose, return a plan path
        self.add_object()  # add ground
        group = self.group

        # set the target position [orientation x, orientation y, orientation z, 
        #                             position x,  position y,  position z,]
        quaternion = quaternion_from_euler(p[0],p[1], p[2])
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = p[3]
        pose_goal.position.y = p[4]
        pose_goal.position.z = p[5]
        # generate the plan
        plan = group.plan(pose_goal)
        group.stop
        return plan

    def cartesian_path(self, anglex, angley, anglez, xscale,yscale,zscale):
        # input orientaions and the scale value to get a cartesian path
        self.add_object()  # add ground
        group = self.group

        # set a list of waypoints for the end-effector to go through
        waypoints=[]
        #group.set_start_state_to_current_state()
        pose=geometry_msgs.msg.Pose()
        pose=group.get_current_pose().pose
        quaternion = quaternion_from_euler(anglex, angley, anglez)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        pose.position.x +=xscale*1
        pose.position.y +=yscale*1
        pose.position.z +=zscale*1
        waypoints.append(copy.deepcopy(pose))
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        group.stop
        return plan
        
if __name__ == "__main__":
    q = UR5Control()
    rospy.spin()
