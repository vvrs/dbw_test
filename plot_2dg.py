#!/usr/bin/env python

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import rospy
from nav_msgs.msg import Odometry
#from nav_msgs.msg import Odometry
import numpy as np
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Vector3Stamped

points = []

x = 0
while x<=2*np.pi:
    # points.append((40*((1-np.sin(x))*np.cos(x)),40*(1-np.sin(x))))
    points.append((40*x*np.sin(x),40*x*np.cos(x)))
    x += 0.05


i=0
points = np.array(points)
plt.ion()
class DynamicUpdate():
    def __init__(self,points):
        global i
        self.xdata=[]
        self.xdata1=[]
        self.ydata=[]
        self.ydata1=[]
        self.x = None
        self.y = None
        self.points = points

        # print self.points
        self.fig, self.ax = plt.subplots(1, 1)
        self.ax.axis('equal')
        self.ax.plot(self.points[:,0],self.points[:,1])
        if(bool(len(self.xdata))):
        	self.ax.plot(self.xdata[-1],self.ydata[-1],'ro')
        self.lines, = self.ax.plot(self.xdata,self.ydata,color='g', linewidth=2.0)
        self.lines1, = self.ax.plot(self.xdata1,self.ydata1,color='r', linewidth=1.0)
        # self.dots, = self.ax.plot([],[],'ro')
        # self.ax.set_autoscaley_on(True)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_ylim([-40.0, 40.0])
        self.ax.set_xlim([-40.0, 40.0])
        self.ax.grid()

    def PlotData(self, x, y,goalx,goaly):
        global i
        # print goalx,goaly
        i+=1
        self.xdata.append(x)
        self.xdata1.append(goalx)
        self.ydata.append(y)
        self.ydata1.append(goaly)
        self.x = x
        self.y = y
        self.lines.set_data(self.xdata,self.ydata)
        self.lines1.set_data(self.xdata1,self.ydata1)
        # self.dots.set_data([goalx],[goaly])
        # self.ax.relim()
        # self.ax.autoscale_view()
        #We need to draw *and* flush
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



rospy.init_node("plot",anonymous=True)
position_vector = rospy.wait_for_message("/vehicle/perfect_gps/utm",Vector3Stamped)

init_x = position_vector.vector.x
init_y = position_vector.vector.y
points = points + np.array([init_x,init_y])
plot = DynamicUpdate(points)


while not rospy.is_shutdown():
    pos=rospy.wait_for_message("/vehicle/perfect_gps/utm",Vector3Stamped)
    goalx = rospy.wait_for_message('/goalx',Float64)
    goaly = rospy.wait_for_message('/goaly',Float64)
    print goalx,goaly
    x=pos.vector.x
    y=pos.vector.y
    plot.PlotData(x,y,goalx.data,goaly.data)
