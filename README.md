# Robot Programming

This repository contains a ROS-based simulated grape-counting robot system that traverses each side of a grape vine to count bunches of grapes.

The robot is based on a real physical robot design in active use at the University of Lincoln, equipped with a range of sensors and four independently steerable wheels.  The simulated robot accurately reflects many of the real world elements, such as obstacle detection using lidar, kinect camera views on three sides (not back) and the environment in this example contains a fairly accurate representation of a planeted vine - with other more complex environments available to test the robot's abilities.

To successfully run the system, test it and see it working, ideally you need to clone the full robprog repository as this contains not only the scripts but also dependencies such as worlds, rviz visualiser, maps and software tools that were used to reach the end stage of development.

The environment calls on files found within the CMP9767M repository which you can set up most easily using the Docker Virtual Desktop and pulling the necessary files from the UoL archive.

Install Docker using this resource: https://docs.docker.com/desktop/ and when running, follow the instructions on the resource here: https://github.com/LCAS/CMP9767M/wiki/using-the-module-resources-in-docker.  This will provide the correct environment with all the secondary associated CMP9767M files referenced by the simulation described on this page.  You still need to clone the robprog (this) repository to install the specific simulation files for this particular roboit grape counter.

Once robprog and CMP9767M directories are in place, if you are running a correctly-configured Docker image, then you can start the simulation as follows:

Type http://localhost:6080/ in a web-browser and launch a terminal.  You will eventually need three separate terminal tabs to launch the necessary a) ROS vineyard "word" b) rviz visualiser/map and c) Python grape-counting components.  

a) So, starting with the vineyard (without gazebo to improve performance), type into a shell tab (or copy/paste): roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small gui:=false this will launch the word and an RVIZ environment that provides an interface between the ROS system and the simulated ROS environment.  You should see a 3D representation of the robot on a grid with a line of grapes (glowing red due to the lidar of the robot).

b) To start the required RVIZ configuration, *in a new tab* type: roslaunch uol_cmp9767m_tutorial topo_nav.launch.  Once this is running, you need to open the *modified* .rviz config file that contains a map; this is called topo_nav.rviz, found in /CMP9767M/uol_cmp9767m_tutorial/config/topo_nav.rviz.  This modified config provides the robot with a map that allows it to navigate around the vine.

c) Finally, the code that controls the grape counting process needs to be located and launched.  This python code is located within scripts.  From a new tab, type (or copy/paste) cd catkin_ws/src/robprog/grape_counter/scripts/. To run the grape-counter (depending on your set up) type either pythin or python3 fruit_counter_main.py. This will launch a window that shows a black screen with some white blobs.  This confirms that the configuration is correct and that the grape counter is running.






