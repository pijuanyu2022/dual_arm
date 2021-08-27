# dual_arm
This is the file of dual_arm


Dual UR5 Robot Arms Assembly with CoppeliaSim and Docker

Description: In this robot simulation, a new object will be assembled by two UR5 arms with RG2 grippers. These two UR5 arms will be controlled by the Moveit!, remote API, python3 and ROS.  The whole simulation will be done in the CoppeliaSim cross-platform. The docker approach will be helpful to make sure that everyone will run the code in the same environment with different machines and operating systems.

Simulator: CoppeliaSim

Robots: UR5 robot arm with RG2 gripper, conveyor belts

Sensors: Proximity ultrasonic cone sensor, Proximity ultrasonic ray sensor 

Software packages: Moveit! 
                                  CoppeliaSim remote API
                                  Universal Robot

Starting point: https://www.youtube.com/watch?v=klTVutd1lLA 

Summary: In this scenario, the right UR5 will pick up a primary component. The left UR5 will pick up four minor different shapes components, which are cylinder, cuboid, X-shaped column and H-shaped column. And then they will collaborate to generate a new object with these components. 
Firstly, they will synchronously pick up the components which are put on a conveyor belt at a random position. Then they will go to the assembly position which is roughly halfway between two UR5 arms. In the third steps, the left UR5 will insert the minor component to the primary component. Then it will go back to pick other components and repeat the process until all components are inserted into the primary component. In the last step, the left arm will go back to the ready position to prepare for the next component. The right arm will put this new component on a third conveyor belt. Finally, the third conveyor belt will take this new object away and the right arm will go back to the ready position.








Quickstart: 
Step 1: Set up the docker environment from this quickstart https://docs.google.com/document/d/1N-un4wxBw6vXQwsOa1k-SFBNmGcGvfSj/edit 
(make sure that you can correctly open the CoppeliaSim and roscore)

Step 2: Open a new terminal, input the command :
xhost +local:root

Step 3: Input the command to open the docker:
sudo docker start -i $YOUR_DOCKER_CONTAINER_NAME

Step 4: Open the byobu session by using this command: byobu 
(F2 is to open a new terminal. F3 is to go to the next terminal. F4 is to go back to the previous terminal. Input “Exit” is to close the terminal) 

Step 5: Use F2 to open a new terminal, and then input : roscore

Step 6: Open a new terminal, use these command to open CoppeliaSim: 
cd ~/YOU_FILE_NAME/CoppeliaSim
./coppeliaSim.sh

Step 7: Open a new terminal, clone files in the src path to get the simulation (go to your workspace/src, then open a terminal and input) :
git clone https://github.com/pijuanyu2022/dual_arm 
git clone https://github.com/pijuanyu2022/universal_robot 

Step 8: In the CoppeliaSim, open the final_demo_beta.ttt file in the dual_arm folder.

Step 9: In your workspace (not in the workspace/src), use: catkin_make

Step 10: Install moveit by using: sudo apt install ros-melodic-moveit
For more installation instructions, go to https://ros-planning.github.io/moveit_tutorials/ 

Step 11: Click the start button in the CoppeliaSim and then input :
roslaunch dual_arm dual_moveit.launch
roslaunch dual_arm total.launch
