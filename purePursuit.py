#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Vector3Stamped, TwistStamped
from dbw_mkz_msgs.msg import ThrottleCmd,SteeringCmd,BrakeCmd,GearCmd


from math import *
import numpy as np
import time
import sys
# import matplotlib.plot as plt
points = []

x = 0
while x< 2*np.pi:
	#points.append((40*((1-sin(x))*cos(x)),(1-sin(x))))
	points.append((40*x*sin(x),40*x*cos(x)))
	x += 0.01

points = np.array(points)


def make_throttle_msg(throttle_value=0.0):

		throttle_command_object = ThrottleCmd()
		throttle_command_object.pedal_cmd = throttle_value
		throttle_command_object.pedal_cmd_type = 2
		throttle_command_object.enable = True
		throttle_command_object.ignore = False
		throttle_command_object.count = 0
		
		return throttle_command_object

def make_steering_msg(angle_rad=0.0):
		
		steering_command_object = SteeringCmd()
		steering_command_object.steering_wheel_angle_cmd = angle_rad
		# steering_command_object.steering_wheel_angle_velocity = random.uniform(0.0, 8.7)
		steering_command_object.steering_wheel_angle_velocity = 5*angle_rad
		
		steering_command_object.enable = True
		steering_command_object.ignore = False
		steering_command_object.quiet = False
		steering_command_object.count = 0
		return steering_command_object

def make_brake_msg(brake_value=0.0):
		
		brake_command_object = BrakeCmd()
		brake_command_object.pedal_cmd = brake_value
		brake_command_object.pedal_cmd_type = 2
		brake_command_object.boo_cmd = False
		brake_command_object.enable = False
		brake_command_object.ignore = False
		brake_command_object.count = 0

		return brake_command_object

class closestPoint:
	def __init__(self, points):
		self.points = points

	def distance(self,p1,p2):
		return sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)

	def closest_node(self, node):
		nodes = self.points
		dist_2 = np.sum((nodes - node)**2, axis=1)
		return np.argmin(dist_2)

class purePursuit:

	def __init__(self,points):

		rospy.loginfo("initializing the node...")
		
		rospy.init_node("purePursuit_node",anonymous=True)

		self.look_ahead_distance = 10

		self.target_speed = 5

		self.k = 0.1
		self.kp = 1

		self.init_xy = False
		self.init_heading = False

		self.car_init_x = None
		self.car_init_y = None 
		self.car_init_heading = None

		self.car_current_x = None
		self.car_current_y = None 
		self.car_current_heading = None
		self.current_speed = None
		self.steering_ratio = rospy.get_param("/vehicle/twist_controller/steering_ratio")
		rospy.Subscriber("/vehicle/perfect_gps/utm",Vector3Stamped,self.callback_utm)
		rospy.Subscriber("/vehicle/perfect_gps/heading",Float64,self.callback_heading)
		rospy.Subscriber("/vehicle/twist",TwistStamped,self.callback_twist)



		time.sleep(0.5)

		self.goalxpub = rospy.Publisher('/goalx',Float64,queue_size=10)
		self.goalypub = rospy.Publisher('/goaly',Float64,queue_size=10)

		self.steering_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
		self.gear_pub = rospy.Publisher('/vehicle/gear_cmd', GearCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

		time.sleep(0.5)

		self.points = points
		

	def callback_utm(self,data):
		if not self.init_xy:
			self.car_init_x = data.vector.x
			self.car_init_y = data.vector.y
			# self.car_current_x = data.vector.x
			# self.car_current_y = data.vector.y
			self.init_xy = True

		else:
			self.car_current_x = data.vector.x - self.car_init_x
			self.car_current_y = data.vector.y - self.car_init_y
		# print "Cureent position:: ",self.car_current_x,self.car_current_y 


	def callback_heading(self,data):

		if not self.init_heading:
			self.car_init_heading = np.deg2rad(data.data) - (np.pi/2)
			# self.car_current_heading = data.data
			self.init_heading = True

		else:
			self.car_current_heading = np.deg2rad(data.data) - (np.pi/2)

	def callback_twist(self,data):
		self.current_speed = data.twist.linear.x

	def _controller(self):

		path = closestPoint(self.points)
		ind = path.closest_node((self.car_current_x,self.car_current_y))

		L = 0
		dind = ind
		print self.current_speed
		Lf = self.k*self.current_speed + self.look_ahead_distance
		while L<Lf and dind < self.points.shape[0]:
			dx = self.points[dind+1][0] - self.points[dind][0]
			dy = self.points[dind+1][1] - self.points[dind][1]

			d = sqrt(dx**2 + dy**2)

			L += d

			dind += 1


		goal_x = self.points[(dind)%len(self.points)][0]
		goal_y = self.points[(dind)%len(self.points)][1]
		near_x = self.points[(ind)%len(self.points)][0]
		near_y = self.points[(ind)%len(self.points)][1]

		self.goalxpub.publish(goal_x)
		self.goalypub.publish(goal_y)

		distance = sqrt((self.car_current_x-goal_x)**2 + (self.car_current_y-goal_y)**2)
		distance2 = sqrt((near_x-goal_x)**2 + (near_y-goal_y)**2)

		print "initial position:: ",self.car_init_x,self.car_init_y 
		print "car current position :: ",self.car_current_x,self.car_current_y
		print "goal current position :: ",goal_x,goal_y
		print "nearest point on the path:: ",near_x,near_y
		print "distance:: ",distance
		print "distance2:: ",distance2

		desired_pose = atan2((goal_y-self.car_current_y),(goal_x-self.car_current_x))

		print "desired_pose:: {0} ({1}) ".format(desired_pose,degrees(desired_pose))
		print "current heading:: {0} ({1}) ".format(self.car_current_heading,degrees(self.car_current_heading))
		error = (desired_pose - self.car_current_heading)

		print "heading error:: {0} ({1}) ".format(error,degrees(error))

		# if error < -3.14159:
		# 	# print "error is below -pi"
		# 	error += 2*3.14159
		# elif error > 3.14159:
		# 	# print "error is above pi"
		# 	error -= 2*3.14159
		# else:
		# 	# print "error is in between -pi and pi"
		# 	pass

		L = 2.6
		ld = 10
		kl = 1
		self.throttle = self._PIDControl(self.target_speed,self.current_speed)
		# self.throttle = min(self.throttle,0.2)
		# self.steering = np.arctan(2*L*sin(error)/(kl*self.throttle))
		self.steering = self.steering_ratio * np.arctan2(2*L*sin(error),Lf)
		# self.steering = error


	def _PIDControl(self,target, current):
		a = self.kp * (target - current)
		return a

	def publish_(self):

		self._controller()

		ai = self.throttle
		di = self.steering 

		print "throttle and steering angle :: ",ai,di
		print "\n\n"

		throttle_msg = make_throttle_msg(ai)
		steering_msg = make_steering_msg(di)
		brake_msg = make_brake_msg()
		self.throttle_pub.publish(throttle_msg)
		self.steering_pub.publish(steering_msg)
		self.brake_pub.publish(brake_msg)

		rospy.on_shutdown(self.stopOnShutdown)
	def stopOnShutdown(self):
		ai = 0
		di = 0

		throttle_msg = make_throttle_msg(ai)
		steering_msg = make_steering_msg(di)
		brake_msg = make_brake_msg(1)
		self.throttle_pub.publish(throttle_msg)
		self.steering_pub.publish(steering_msg)
		self.brake_pub.publish(brake_msg)

		time.sleep(1)
		reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
		reset_world()
		print "exiting..."
		return

def main():
	global points
	controller_object = purePursuit(points)

	r = rospy.Rate(50)

	while not rospy.is_shutdown():
		controller_object.publish_()
		r.sleep()

if __name__ == '__main__':
	main()