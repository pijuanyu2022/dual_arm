#!/usr/bin/env python
import rospy
import time
import math
import sys
import copy
import sim
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from robot_control import RobotControl
from dual_arm.msg import Movement

# connect to remote API server for the Right robot arm (Left robot arm will be connested with another remote API server)
clientID=sim.simxStart('127.0.0.1',20002,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')

# get the joint handles for six joints
jointhandles={}
for i in range (0,7):
    er,jointhandles[i]=sim.simxGetObjectHandle(clientID,('Right_UR5_joint'+str(i+1)),sim.simx_opmode_blocking)

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


        # main control loop
        while not rospy.is_shutdown():
            # subscribe the movement message, when the movement message is 5, 
            # the right robot arm will go from the assembly position to the end position
            Movement = self.get_coppeliasim_Value('Movement_val')
            # read sensor values
            V_Right_RG2_front1 = self.get_coppeliasim_Value('Right_RG2_front1')
            V_Right_RG2_front2 = self.get_coppeliasim_Value('Right_RG2_front2')
            V_Right_RG2_touch = self.get_coppeliasim_Value('Right_RG2_touch')

            # Step 1 : when the front sensors does not detect anything
            # the right arm go to the assemblt position
            if V_Right_RG2_front1 < 0.01 and V_Right_RG2_front2 < 0.01 and V_Right_RG2_touch < 0:

                # the first three numebers are the orientation
                # the last three numbers are the position
                pickuppos=[math.pi/2, math.pi, math.pi/2, 0.3652, -0.3562, 0.228]

                # generate the path
                plan = self.plan_path(pickuppos)
                # go to the position, 0.02 means the gripper will open
                self.execute_cartesian(plan, 0.02)

            # Step 2 : when the object is detected by the front sensor
            # the left arm go to the ready position
            if V_Right_RG2_front1 > 0.01 and V_Right_RG2_front2 > 0.01 and V_Right_RG2_touch < 0:
                
                # close to the object
                A = V_Right_RG2_front1-0.03
                plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, 0,-A,0)
                self.execute_cartesian(plan, 0.02)

                # pick up the object, -0.05 means the gripper will close
                plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, 0,0,0)
                self.execute_cartesian(plan, -0.05)

                # go up for stable
                plan = self.cartesian_path(math.pi/2, math.pi, math.pi/2, 0,0,0.1)
                self.execute_cartesian(plan, -0.05)

            # Step 3 : when the movement have not been published by the left robot arm, 
            # the right arm go to the assemble position
            if V_Right_RG2_front1 > 0.01 and V_Right_RG2_front2 > 0.01 and V_Right_RG2_touch > 0 and Movement < 4 :
                
                
                #pickuppos = [3*math.pi/2, 0, math.pi/2, -0.11, 0.639, 0.288]
                p2 = [math.pi/2, math.pi, 3*math.pi/2, -0.11, 0.709, 0.325] #0.350
                # go to the assemblt position
                plan = self.plan_path(p2)
                self.execute_cartesian(plan, -0.05)

            # Step 4 : when the movement have been published by the left robot arm, 
            # the right arm go from the assemble position to the end position
            if V_Right_RG2_front1 > 0.01 and V_Right_RG2_front2 > 0.01 and Movement > 4 :

                # go to the end position
                plan = self.cartesian_path(math.pi/2, math.pi, 4*math.pi/2, -0.6, -0.6, 0)
                self.execute_cartesian(plan, -0.05)

                # go down
                plan = self.cartesian_path(math.pi/2, math.pi, 4*math.pi/2,   0, 0, -0.09)
                self.execute_cartesian(plan, -0.05)

                # open the gripper
                plan = self.cartesian_path(math.pi/2, math.pi, 4*math.pi/2,   0, 0, -0.02)
                self.execute_cartesian(plan, 0.05)

                # stay away from this object
                plan = self.cartesian_path(math.pi/2, math.pi, 4*math.pi/2,   0.15, 0, 0)
                self.execute_cartesian(plan, 0.05)
                
                # go back to the ready position to prepare for more objects
                pickuppos=[math.pi/2, math.pi, math.pi/2, 0.3652, -0.3562, 0.228]
                plan = self.plan_path(pickuppos)
                self.execute_cartesian(plan, 0.02)

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