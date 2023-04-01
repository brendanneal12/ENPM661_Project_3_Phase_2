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

Recommended IDE: Visual Studio Code

Python Version: 3

# GitHub Repository Links

Brendan: https://github.com/brendanneal12/ENPM661_Project_3_Phase_2

Adam: https://github.com/AdazInventionz/ENPM661-Project-3-Part-2

# Libraries Used
numpy, opencv, from matplotlib: pyplot, math, timeit, from queue: PriorityQueue, csv, rospy

# How to Run Code: Part 1
1. Hit Run
2. Prompted by the terminal, enter the initial state X, Y, and Theta with spaces separating them. Example: 50 100 0
3. Prompted by the terminal, enter the initial state X and Y, with spaces separating them. Example: 500 100
4. Prompted by the terminal, enter your desired clearance from obstacles. Example: 5
5. Prompted by the terminal, enter 2 unique wheel RPMS, separated by spaces. Example: 12 10
6. If your initial or goal state is inside an obstacle or outside of the workspace, you will be prompted to restart.
7. Observe the obstacle space setup. Obstacles are white, the entered desired clearance is light gray, and the additional robot radius is dark gray.
8. Close the window to begin the A* search.
9. While the search is running, the currend node state (popped from the open list) is printed to the terminal.
10. Once the goal is reached, the total cost and time to complete search will be printed to the terminal. The final map will be displayed to the screen with the searched nodes in green, and the optimal path in magenta.
11. Additionally, a .csv file will appear in your workspace. This will be used later in Phase 2 Part 2.
12. Close the window to start the visualization.
13. Once the visualization is complete, close the window to end the program.

# Part 1 Important Notes
1. For the 2-D visualization, we defined everything in CENTIMETERS to better visualize on our screen.
2. For cases far away from the start point, the search can take upward of 10-15 minutes. We are not generating repeat nodes. This long search time is due to us using OOP the process is computationally expensive. Please be patient.
3. The visualization is also quite slow. Please be patient.
4. Please click the following link in order to view an example output video. Initial State is 50 100 0. Goal State is 500 100. Desired Clearance is 5 and the wheel RPMS are 12 10. I cannot commit the video directly to GitHub since the file size is too large: https://drive.google.com/drive/u/0/folders/1An1wukgfk4Zb2PrgWcUQ8Ll5RnvF3wBx

# How to Run Code: Part 2

# Part 2 Important Notes

