# robotic-programming

After updating!!! Yay

Make Sure Docker (Mac Desktop App) is running (looks like a whale icon)

THEN 

Need to start the ROS environment using Docker...

Navigate to the robprog directory (on the Mac interface) type: docker-compose up

REMEMBER: at the end of the session, CTRL-C thentype docker-control down to ensure it is not still running in the background.


Type http://localhost:6080/ in a web-browser

Use http://localhost:6081/ for the code editor

Launch LXTERMINAL from the virtual environment.

Type roslaunch bacchus_gazebo vineyard_demo.launch

More complex environment with different plant stages:

roslaunch bacchus_gazebo vineyard_demo.launch world_name:=<WORLD>
with <WORLD> being one of the following:

vineyard_small
  
vineyard_small_s0_coarse
  
vineyard_small_s1_coarse
vineyard_small_s2_coarse
vineyard_small_s3_coarse
vineyard_small_s4_coarse
vineyard_stage0
vineyard_stage0_heightmap
vineyard_stage0_small
vineyard_stage1
vineyard_stage2
vineyard_stage3
vineyard_stage4
vineyard_stage4_small

# Setting up a catkin package
  
Navigate to your folder that will contain your package...
(in my case this is called cmp9767_ws (for workspace)

Type:
$ source /opt/ros/noetic/setup.bash
  
$ mkdir -p ~/catkin_ws/src
  
$ cd ~/catkin_ws/
  
$ catkin_make
  
Now use (in this example) http://localhost:6081/?folder=/home/ubuntu/cmp9767_ws to develop the packages in a way that they can be exported correctly.
 
Now - you need to overlay your workspace on top of the environment so navigate to 'devel' folder and type 
  "source setup.sh" and check that the $ROS_PACKAGE_PATH includes the directory you are in.
  
  
