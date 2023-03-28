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
def setup(robotradius):

    global arena

    #Colors
    white = (255, 255, 255)
    gray = (177, 177, 177)
    darkGray = (104, 104, 104)
    
    #Draw Radial Clearance
    for x in range(0, 600):

        for y in range(0, 250):
        
            if checkClearance(x, y, robotradius):
                arena[y, x] = darkGray
    
    #Draw Obstacle Borders
    for x in range(0, 600):

        for y in range(0, 250):
        
            if checkBorder(x, y):
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
def checkBorder(x, y):
    
    triHeight = int(round(5/math.cos(math.radians(63.4))))
    hexHeight = int(round(5/math.cos(math.radians(30.3))))
    
    #Both Rectangles
    if x >= 100 - 5 and x <= 150 + 5:
        
        if y < 100 + 5 or y >= 150 - 5:
            return True
    
    #Pentagon (Left Half)
    if x >= 235 - 5 and x <= 300:
        
        if (y >= (-38/65)*x + (2930/13) - hexHeight) and (y <= (38/65)*x + (320/13) + hexHeight):
            return True
    
    #Pentagon (Right Half)
    if x >= 300 and x <= 366 + 5:
        
        if (y >= (38/65)*x + (-1630/13) - hexHeight) and (y <= (-38/65)*x + (4880/13) + hexHeight):
            return True
    
    #Triangle
    if x >= 460 - 5 and x <= 510 + 5:
        
        if (y >= 2*x - 895 - triHeight) and (y <= -2*x + 1145 + triHeight) and (y >= 25 - 5) and (y <= 225 + 5):
            return True
        
    return False

##-------------------------------Defining Radial Clearance Function--------------##
def checkClearance(x, y, r):
    
    rr = r+1
    
    if rr == 0:
        return False
    
    triHeight = int(round((5 + rr)/math.cos(math.radians(63.4))))
    hexHeight = int(round((5 + rr)/math.cos(math.radians(30.3))))
    
    #Both Rectangles
    if x >= 100 - 5 - rr and x <= 150 + 5 + rr:
        
        if y < 100 + 5 + rr or y >= 150 - 5 - rr:
            return True
    
    #Pentagon (Left Half)
    if x >= 235 - 5 - rr and x <= 300:
        
        if (y >= (-38/65)*x + (2930/13) - hexHeight) and (y <= (38/65)*x + (320/13) + hexHeight):
            return True
    
    #Pentagon (Right Half)
    if x >= 300 and x <= 366 + 5 + rr:
        
        if (y >= (38/65)*x + (-1630/13) - hexHeight) and (y <= (-38/65)*x + (4880/13) + hexHeight):
            return True
    
    #Triangle
    if x >= 460 - 5 - rr and x <= 510 + 5 + rr:
        
        if (y >= 2*x - 895 - triHeight) and (y <= -2*x + 1145 + triHeight) and (y >= 25 - 5 - rr) and (y <= 225 + 5 + rr):
            return True
        
    return False

##---------------------------------------Defining Check Valid Move Function-----------------------##
#Checks to see if a point is valid (by checking obstacle, border, and clearance, as well as making sure the point is within arena bounds)
def checkValid(x, y, r):
    
    if checkObstacle(x, y):
        return False
    
    if checkBorder(x, y):
        return False
    
    if checkClearance(x, y, r):
        return False
    
    if (x < 0 or x >= 600 or y < 0 or y >= 250):
        return False
    
    return True



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
    print("Enter Wheel RPMS (Left First then Right).")
    WheelRPMS = [int(x) for x in input().split()]
    return  WheelRPMS
