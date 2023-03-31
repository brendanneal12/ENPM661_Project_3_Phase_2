import matplotlib.pyplot as plt
import numpy as np
import math

fig, ax = plt.subplots()

def plot_curve(Xi,Yi,Thetai,UL,UR):
    t = 0
    WheelRad = 0.038
    WheelDist = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180


# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    t = 0
    dt = 0.1
    Curr_Node_X = Xn
    Curr_Node_Y = Yn
    Curr_Node_Theta = np.deg2rad(Thetai)

    New_Node_X = Curr_Node_X
    New_Node_Y = Curr_Node_Y
    New_Node_Theta = Curr_Node_Theta

    while t < 1:
        t += dt
        X_Start = New_Node_X
        Y_Start = New_Node_Y
        New_Node_X += 0.5*WheelRad*(UL+UR)*np.cos(New_Node_Theta)*dt
        New_Node_Y += 0.5*WheelRad*(UL+UR)*np.sin(New_Node_Theta)*dt
        New_Node_Theta += (WheelRad/WheelDist)*(UR-UL)*dt
        plt.plot([X_Start, New_Node_X], [Y_Start, New_Node_Y], color = 'g', linewidth = 0.75)


    return Xn, Yn, Thetan
    

actions=[[5,5], [10,10],[5,0],[0,5],[5,10],[10,5], [10,0], [0,10]]
        
for action in actions:
   X1= plot_curve(0,0,45, action[0],action[1]) # (0,0,45) hypothetical start configuration
   print(X1)
   for action in actions:
      X2=plot_curve(X1[0],X1[1],X1[2], action[0],action[1])
      print(X2)
      
   

  

plt.grid()

ax.set_aspect('equal')

plt.xlim(0,1)
plt.ylim(0,1)

plt.title('How to plot a vector in matplotlib ?',fontsize=10)

plt.show()
plt.close()
    
