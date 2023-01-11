# Robot Programming

This repository contains a ROS-based simulated grape-counting robot system that traverses each side of a grape vine to count bunches of grapes.

The robot is based on a real physical robot design in active use at the University of Lincoln, equipped with a range of sensors and four independently steerable wheels.  The simulated robot accurately reflects many of the real world elements, such as obstacle detection using lidar, kinect camera views on three sides (not back) and the environment in this example contains a fairly accurate representation of a planeted vine - with other more complex environments available to test the robot's abilities.

To successfully run the system, test it and see it working, ideally you need to clone the full robprog repository as this contains not only the scripts but also dependencies such as worlds, rviz visualiser, maps and software tools that were used to reach the end stage of development.

The environment calls on files found within the CMP9767M repository which you can set up most easily using the Docker Virtual Desktop and pulling the necessary files from the UoL archive.

Install Docker using this resource: https://docs.docker.com/desktop/ and when running, follow the instructions on the resource here: https://github.com/LCAS/CMP9767M/wiki/using-the-module-resources-in-docker.  This will provide the correct environment with all the secondary associated CMP9767M files referenced by the simulation described on this page.  You still need to clone the robprog (this) repository to install the specific simulation files for this particular roboit grape counter.

Once robprog and CMP9767M directories are in place, if you are running a correctly-configured Docker image, then you can start the simulation as follows:

Type http://localhost:6080/ in a web-browser and launch a terminal.  You will eventually need three separate terminal tabs to launch the necessary a) ROS vineyard "word" b) rviz visualiser/map and c) Python grape-counting components.  

a) So, starting with the vineyard (without gazebo to improve performance), type into a shell tab (or copy/paste): roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small gui:=false 

<img src="https://github.com/andieroid/robprog/blob/main/grape_counter/scripts/images/launching-vineyard-world.png">

this will launch the word and an RVIZ environment that provides an interface between the ROS system and the simulated ROS environment.  You should see a 3D representation of the robot on a grid with a line of grapes (glowing red due to the lidar of the robot) - see the image below.

<img src="https://github.com/andieroid/robprog/blob/main/grape_counter/scripts/images/initial-rviz-launch.png">

b) To start the required RVIZ configuration, *in a new tab* type: roslaunch uol_cmp9767m_tutorial topo_nav.launch.  Once this is running, you need to open the *modified* .rviz config file that contains a map; this is called topo_nav.rviz, found in /CMP9767M/uol_cmp9767m_tutorial/config/topo_nav.rviz.  

<img src="https://github.com/andieroid/robprog/blob/main/grape_counter/scripts/images/selecting-the-modified-config-file.png">
This modified config provides the robot with a map that allows it to navigate around the vine.

c) Finally, the code that controls the grape counting process needs to be located and launched.  This python code is located within scripts.  From a new tab, type (or copy/paste) cd catkin_ws/src/robprog/grape_counter/scripts/. To run the grape-counter (depending on your set up) type either python or python3 fruit_counter_main.py. This will launch a window that shows a black screen with some white blobs (see image below).  This confirms that the configuration is correct and that the grape counter is running.

<img src="https://github.com/andieroid/robprog/blob/main/grape_counter/scripts/images/black-screen-with-white-blobs.png">

Return to the topo_nav RVIZ environment where you will see the robot on the grid now has a map overlaid with arrows (see image below):

The first step in grape counting is to position the robot at the nearest waypoint (shown as green arrows) around the grape vine.  If your robot happened to be at the top end of the image, you can click the nearest waypoint, however by default, the robot appears at the bottom of the screen, so click the waypoint as shown in the image below; the robot should move into position, ready to start counting.

The robot is programmed to follow the map layout, which has a one way system, so if you click the bottom right waypoint (the final destination - see image below) it should move up the left side of the vine, move across at the top and come back down the other side, counting the grapes as it goes. 

To see the counting process, once you have clicked the final waypoint, you need to click the tab at the bottom to open the "Grape Bunch Counter" window. The white blobs represent the grapes that the robot is seeing.  There should be one or more red-bounded shapes with green writing.  These represent the grapes that will be tracked untl they reach the right-hand side of the screen, so that the robot knows when a "batch" can be counted.

At the end of the process, the result will be displayed in the terminal tab that was used to launch the grape counter.  Re-open this tab to see how many grapes the robot counted.







