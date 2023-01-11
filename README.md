# Robot Programming

This repository contains a ROS-based simulated grape-counting robot system that traverses each side of a grape vine to count bunches of grapes.

The robot is based on a real physical robot design in active use at the University of Lincoln, equipped with a range of sensors and four independently steerable wheels.  The simulated robot accurately reflects many of the real world elements, such as obstacle detection using lidar, kinect camera views on three sides (not back) and the environment in this example contains a fairly accurate representation of a planeted vine - with other more complex environments available to test the robot's abilities.

To successfully run the system, test it and see it working, ideally you need to clone the full robprog repository as this contains not only the scripts but also dependencies such as worlds, rviz visualiser, maps and software tools that were used to reach the end stage of development.

The environment calls on files found within the CMP9767M repository which you can set up most easily using the Docker Virtual Desktop and pulling the necessary files from the UoL archive.

Install Docker using this resource: https://docs.docker.com/desktop/ and when running, follow the instructions on the resource here: https://github.com/LCAS/CMP9767M/wiki/using-the-module-resources-in-docker.  This will provide the correct environment with all the secondary associated CMP9767M files referenced by the simulation described on this page.  You still need to clone the robprog (this) repository to install the specific simulation files for this particular roboit grape counter.

Once robprog and CMP9767M directories are in place, if you are running a correctly-configured Docker image, then you can start the simulation as follows:

Type http://localhost:6080/ in a web- and launch a terminal.  You will need three separate tabs to launch the necessary a) ROS vineyard "word" b) rviz visualiser/map and c) Python grape-counting components.  

a) So, starting with the vineyard (without gazebo to improve performance), type into a shell tab (or copy/paste): roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small gui:=false

b) To start the RVIZ component, that provides an interface between the ROS system and the simulated ROS environment, *in a new tab* type: roslaunch uol_cmp9767m_tutorial topo_nav.launch
Once this is running (with a 3D representation of the robot on a grid with a line of grapes (glowing red due to the lidar of the robot), open the config file named topo_nav.rviz.  This modified config provides the robot with a map that allows it to navigate around the vine.

c) Finally, the code that controls the grape counting element needs to be located and launched.  This python code is located within scripts.  From a new tab, type (or copy/paste) cd catkin_ws/src/robprog/grape_counter/scripts/. To run the grape-counter (depending on your set up) type either pythin or python3 fruit_counter_main.py. This will launch a window that shows a black screen with some white blobs.  This confirms that the configuration is correct.










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
  
  
