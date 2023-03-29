## Brendan Neal and Adam Lobo
##ENPM661 Project 3 Phase 2 Part 1 Main Script

##------------------------Importing Libraries-------------------------##
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import math
import timeit
import queue
from queue import PriorityQueue


##------------Defining Node Class (From Previous Project)-----------------##

class Node():
    #Initializing Function
    def __init__(self, state, parent, move, C2C, TotalCost):
        self.state = state
        self.parent = parent
        self.move = move
        self.C2C = C2C
        self.TotalCost = TotalCost

    #---Methods for this Class---#
    def ReturnState(self): #Returns Node State X and Y
        return self.state
    
    def ReturnParent(self): #Returns the Parent Node
        return self.parent
    
    def ReturnParentState(self): #Returns the Parent Node's State
        if self.ReturnParent() is None:
            return None
        return self.ReturnParent().ReturnState()
    
    def ReturnMove(self): #Returns Move
        return self.move
    
    def ReturnC2C(self): # Returns C2C
        return self.C2C
    
    def ReturnTotalCost(self): #Returns the Total Cost
        return self.TotalCost

    def __lt__(self, other): #OOP Definition for Less than. Required for Priority Queue.
        return self.TotalCost < other.TotalCost
    
    ##--------------BACKTRACKING FUNCTION Integrated into Class--------##
    def ReturnPath(self):
        CompletedMoves = [] #Initialize Move Array
        NodePath = [] #Initialize the Node Path
        CurrentNode = self
        while(CurrentNode.ReturnMove() is not None): #For move that a Node has made
            CompletedMoves.append(CurrentNode.ReturnMove()) #Append the previous move
            NodePath.append(CurrentNode) #Append Node to Path
            CurrentNode = CurrentNode.ReturnParent() #Backtrack to the Parent before repeating Process
        NodePath.append(CurrentNode) #Append the starting point after path is derived.
        NodePath.reverse() #Reverse Order to get front to back path
        CompletedMoves.reverse() #Reverse Order to get front to back path

        return CompletedMoves, NodePath
    
##----------------------Defining Obstacle Space Setup Functions--------------------##
def setup(robotradius, ObsClearance):

    global arena

    #Colors
    white = (255, 255, 255)
    gray = (177, 177, 177)
    darkGray = (104, 104, 104)
    
    #Draw Radial Clearance
    for x in range(0, 600):

        for y in range(0, 250):
        
            if checkClearance(x, y, robotradius, ObsClearance):
                arena[y, x] = darkGray
    
    #Draw Obstacle Borders
    for x in range(0, 600):

        for y in range(0, 250):
        
            if checkBorder(x, y, ObsClearance):
                arena[y, x] = gray
    
    #Draw Obstacles
    for x in range(0, 600):

        for y in range(0, 250):
        
            if checkObstacle(x, y):
                arena[y, x] = white
                
##---------------------------Obstacle Setup Function------------------------##
def checkObstacle(x, y):
    
    #Both Rectangles
    if x >= 100 and x <= 150:
        
        if y < 100 or y >= 150:
            return True
    
    #Pentagon (Left Half)
    if x >= 235 and x <= 300:
        
        if (y >= (-38/65)*x + (2930/13)) and (y <= (38/65)*x + (320/13)):
            return True
    
    #Pentagon (Right Half)
    if x >= 300 and x <= 366:
        
        if (y >= (38/65)*x + (-1630/13)) and (y <= (-38/65)*x + (4880/13)):
            return True
    
    #Triangle
    if x >= 460 and x <= 510:
        
        if (y >= 2*x - 895) and (y <= -2*x + 1145):
            return True
        
    return False
  
##-----------------------------Border Check Function---------------------##
def checkBorder(x, y, ObsClearance):
    
    triHeight = int(round(5/math.cos(math.radians(63.4))))
    hexHeight = int(round(5/math.cos(math.radians(30.3))))
    
    #Both Rectangles
    if x >= 100 - ObsClearance and x <= 150 + ObsClearance:
        
        if y < 100 + ObsClearance or y >= 150 - ObsClearance:
            return True
    
    #Pentagon (Left Half)
    if x >= 235 - ObsClearance and x <= 300:
        
        if (y >= (-38/65)*x + (2930/13) - hexHeight) and (y <= (38/65)*x + (320/13) + hexHeight):
            return True
    
    #Pentagon (Right Half)
    if x >= 300 and x <= 366 + ObsClearance:
        
        if (y >= (38/65)*x + (-1630/13) - hexHeight) and (y <= (-38/65)*x + (4880/13) + hexHeight):
            return True
    
    #Triangle
    if x >= 460 - ObsClearance and x <= 510 + ObsClearance:
        
        if (y >= 2*x - 895 - triHeight) and (y <= -2*x + 1145 + triHeight) and (y >= 25 - ObsClearance) and (y <= 225 + ObsClearance):
            return True
        
    return False

##-------------------------------Defining Radial Clearance Function--------------##
def checkClearance(x, y, r, ObsClearance):
    
    rr = r+1
    
    if rr == 0:
        return False
    
    triHeight = int(round((5 + rr)/math.cos(math.radians(63.4))))
    hexHeight = int(round((5 + rr)/math.cos(math.radians(30.3))))
    
    #Both Rectangles
    if x >= 100 - ObsClearance - rr and x <= 150 + ObsClearance + rr:
        
        if y < 100 + ObsClearance + rr or y >= 150 - ObsClearance - rr:
            return True
    
    #Pentagon (Left Half)
    if x >= 235 - ObsClearance - rr and x <= 300:
        
        if (y >= (-38/65)*x + (2930/13) - hexHeight) and (y <= (38/65)*x + (320/13) + hexHeight):
            return True
    
    #Pentagon (Right Half)
    if x >= 300 and x <= 366 + 5 + rr:
        
        if (y >= (38/65)*x + (-1630/13) - hexHeight) and (y <= (-38/65)*x + (4880/13) + hexHeight):
            return True
    
    #Triangle
    if x >= 460 - ObsClearance - rr and x <= 510 + ObsClearance + rr:
        
        if (y >= 2*x - 895 - triHeight) and (y <= -2*x + 1145 + triHeight) and (y >= 25 - ObsClearance - rr) and (y <= 225 + ObsClearance + rr):
            return True
        
    return False

##---------------------------------------Defining Check Valid Move Function-----------------------##
#Checks to see if a point is valid (by checking obstacle, border, and clearance, as well as making sure the point is within arena bounds)
def checkValid(x, y, r, ObsClearance):
    
    if checkObstacle(x, y):
        return False
    
    if checkBorder(x, y, ObsClearance):
        return False
    
    if checkClearance(x, y, r, ObsClearance):
        return False
    
    if (x < 0 or x >= 600 or y < 0 or y >= 250):
        return False
    
    return True


##---------------------------------Defining my Action Set-----------------------------------------##

def ReturnPossibleStates(CurrentNodeState, Wheel_RPMS, RobotRadius, ObsClearance, WheelRad, WheelDist):
    RPM1 = Wheel_RPMS[0]
    RPM2 = Wheel_RPMS[1]
    ActionSet = [[RPM1, RPM1], [RPM2,RPM2],[RPM1, RPM2], [RPM2, RPM1], [0,RPM1], [RPM1,0], [0,RPM2], [RPM2,0]]
    NewNodeStates = []

    for action in ActionSet:
        NewNodeState, Cost = CalcMoveWithCost(CurrentNodeState, action, RobotRadius, ObsClearance, WheelRad, WheelDist)
        if NewNodeState is not None:
            NewNodeStates.append(NewNodeState)

    return NewNodeStates

##---------------------------------Defining my Cost Function--------------------------------------##

def CalcMoveWithCost(CurrentNodeState, WheelAction, RobotRadius, ObsClearance, WheelRad, WheelDist):
    t = 0
    dt = 0.1
    Curr_Node_X = CurrentNodeState[0]
    Curr_Node_Y = CurrentNodeState[1]
    Curr_Node_Theta = np.deg2rad(CurrentNodeState[2])
    Cost = 0
    New_Node_X = Curr_Node_X
    New_Node_Y = Curr_Node_Y
    New_Node_Theta = Curr_Node_Theta
    while t < 1:
        t += dt
        ChangeX = 0.5*WheelRad*(WheelAction[0]+WheelAction[1])*np.cos(Curr_Node_Theta)*dt
        ChangeY = 0.5*WheelRad*(WheelAction[0]+WheelAction[1])*np.sin(Curr_Node_Theta)*dt
        ChangeTheta = (WheelRad/WheelDist)*(WheelAction[0]-WheelAction[1])*dt
        New_Node_X += ChangeX
        New_Node_Y += ChangeY
        New_Node_Theta += ChangeTheta

        Cost += np.sqrt((ChangeX)**2 + (ChangeY)**2)

        if checkValid(New_Node_X, New_Node_Y, RobotRadius, ObsClearance) == False:
            return None, None
        
        New_Node_Theta = np.rad2deg(New_Node_Theta)

        if New_Node_Theta >= 360:
            New_Node_Theta = New_Node_Theta - 360
        if New_Node_Theta < 0:
            New_Node_Theta = New_Node_Theta + 360

        return [New_Node_X, New_Node_Y, New_Node_Theta], Cost



##--------------------------Defining my Plotting Functions--------------------------##
'''For Integers'''
def WSColoring(Workspace, Location, Color):
    x,_,_ = Workspace.shape #Get Shape of Workspace
    translation_x = Location[0] #Where in X
    translation_y = Location[1] #Where in Y
    Workspace[translation_x,translation_y,:] = Color #Change the Color to a set Color
    return Workspace  



##------------------------Defining my GetInitialState Function-----------------------##
def GetInitialState():
    print("Enter Initial Node X, Y, and Theta separated by spaces: ")
    Init_State=[int(x) for x in input().split()]
    return Init_State

##------------------------Defining my GetGoalState Function--------------------------##
def GetGoalState():
    print("Enter Goal Node X and Y, separated by spaces: ")
    Goal_State=[int(x) for x in input().split()]
    return  Goal_State
##-------------------------Defining my Get Robot Radius Function---------------------##
def GetClearance():
    print("Enter Desired Clearance From Obstacles.")
    Clearance=int(input())
    return  Clearance

##--------------------------Defining my GetWheelRPMS Function------------------------##
def GetWheelRPM():
    print("Enter Wheel RPMS, 2 Unique, Separated By Spaces")
    WheelRPMS = [int(x) for x in input().split()]
    return  WheelRPMS



##----------------------------------"Main" Script-------------------------------------##

##-------Getting Parameters from Burger TurtleBot Dimensions-------##

WheelRadius = 33 #mm
RobotRadius = 89 #mm
WheelDistance = 160 #mm

##----------------------Arena Setup-------------------##
arena = np.zeros((250, 600, 3), dtype = "uint8")
InitState = GetInitialState()
GoalState =GetGoalState()
DesClearance = GetClearance()
WheelRPMS = GetWheelRPM()

# if not checkValid(InitState[0], InitState[1], RobotRadius, DesClearance):
#     print("Your initial state is inside an obstacle or outside the workspace. Please retry.")
#     exit()
# if not checkValid(GoalState[0], GoalState[1], RobotRadius, DesClearance):
#     print("Your goal state is inside an obstacle or outside the workspace. Please retry.")
#     exit()

setup(RobotRadius, DesClearance)

WSColoring(arena, InitState, (0,255,0))
WSColoring(arena, GoalState, (0,255,0))

plt.imshow(arena, origin='lower')
plt.show()

#Initialize Arena and Thresholds
SizeArenaX = 600
SizeArenaY = 250
ThreshXY = 0.5
ThreshTheta = 30
ThreshGoalState = 1.5

# Initialize Node Array
node_array = np.array([[[ 0 for k in range(int(360/ThreshTheta))] 
                        for j in range(int(SizeArenaX/ThreshXY))] 
                        for i in range(int(SizeArenaY/ThreshXY))])
