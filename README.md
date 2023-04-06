# ENPM661_Project_3_Phase_2
UMD ENPM661 Project 3 Phase 2.

# Student Information
Brendan Neal:

Directory ID: bneal12.

UID: 119471128.

Adam Lobo:

Directory ID: alobo.

UID: 115806078.

# Project Information
Goal: Solve a Maze using the A* Algorithm with Differential Drive Non-Holonomic Constraints

File Names: a_star_adam_brendan_diffdrive.py, ROS Package (inside zip folder)

Recommended IDE: Visual Studio Code

Python Version: 3

# GitHub Repository Links

Brendan: https://github.com/brendanneal12/ENPM661_Project_3_Phase_2

Adam: https://github.com/AdazInventionz/ENPM661-Project-3-Part-2

# Libraries Used
numpy, opencv, from matplotlib: pyplot, math, timeit, from queue: PriorityQueue, csv, rospy

# How to Run Code: Part 1
1. Download a_star_adam_brendan_diffdrive.py, then cd to your your download location and type "python3 a_star_adam_brendan_diffdrive.py" to start the code.
2. Prompted by the terminal, enter the initial state X, Y, and Theta with spaces separating them. Example: 50 100 0
3. Prompted by the terminal, enter the goal state X and Y, with spaces separating them. Example: 500 100
4. Prompted by the terminal, enter your desired clearance from obstacles. Example: 5
5. Prompted by the terminal, enter 2 unique wheel RPMS, separated by spaces. Example: 12 10
6. If your initial or goal state is inside an obstacle or outside of the workspace, you will be prompted to restart.
7. Observe the obstacle space setup. Obstacles are white, the entered desired clearance is light gray, and the additional robot radius is dark gray.
8. Close the window to begin the A* search.
9. While the search is running, the currend node state (popped from the open list) is printed to the terminal.
10. Once the goal is reached, the total cost and time to complete search will be printed to the terminal. The final map will be displayed to the screen with the optimal path in magenta.
11. Close the window to start the visualization.
12. Once the visualization is complete, close the window to end the program.

# Part 1 Important Notes
1. For the 2-D visualization, we defined everything in centimeters to better visualize on our screen.
2. For cases far away from the start point, the search can take upward of 1-3 minutes. We are not generating repeat nodes. This long search time is due to us using OOP and the process is computationally expensive. Please be patient.
3. The visualization is EXTREMELY slow sometimes. Please be patient.
4. Please click the following link in order to view an example output video. Initial State is 100 25 0. Goal State is 275 175. Desired Clearance is 5 and the wheel RPMS are 12 10. I cannot commit the video directly to GitHub since the file size is too large:


# How to Run Code: Part 2
1. Download the part_2 package to your associated catkin workspace and ensure that you have all the proper turtlebot3 dependencies installed.
2. cd into your catkin workspace and perform catkin_make
3. source devel/setup.bash
4. roslaunch part_2 environment.launch x_init:=CHOOSE y_init:=CHOOSE theta_init:=CHOOSE x_final:=CHOOSE y_final:=CHOOSE clearance:=CHOOSE RPM1:=CHOOSE RPM2:=CHOOSE

EXAMPLE for Cross-Map Exploration: 

roslaunch part_2 environment.launch x_init:=0 y_init:=0 theta_init:=0 x_final:=5 y_final:=0 clearance:=0.1 RPM1:=12 RPM2:=10

5. Like Phase 2 Part 1, while the search occurs, the popped node state will be printed to the terminal.
6. When the search completes, the total cost and time to complete search will be printed to the terminal.
7. Additionally, the terminal will tell you to observe the Gazebo map, so please direct your attention there to watch the 3-D visualization.
8. Once the robot reaches the final destination, it will then crash into a wall and then restart the search. Please spam Ctrl C to end the program.

# Part 2 Important Notes
1. As mentioned in the project description, the robot will not follow the desired waypoints exactly, which is why toward the end the robot will overshoot the goal hit a wall before stopping.
2. For cases far away from the start point, the search can take upward of 1-3 minutes. We are not generating repeat nodes. This long search time is due to us using OOP and the process is computationally expensive. Please be patient.
3. Please click the following link in order to view an example output video. Initial State is 0 0 0. Goal State is 5 0. Desired Clearance is 0.1 and the wheel RPMS are 12 10. I cannot commit the video directly to GitHub since the file size is too large: 

