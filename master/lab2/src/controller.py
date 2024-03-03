#!/usr/bin/env python3

#turtlebot_controller.py
#For lab1, this will subscribe to mouse_client and publish to cmd_vel
#Will convert messages of type MouseController to Twist
#Deactivates when mouse wheel is scrolled up 
#last modified 9 Feb 2024


import rospy
#TODO 2 Import the appropriate message types that we will need
from geometry_msgs.msg import Twist
from lab1.msg import MouseController
from squaternion import Quaternion
from sensor_msgs.msg import Imu

class Controller:
	#TODO 3 initialize the appropriate Controller class attributes
	K_HDG = 0.1 # rotation controller constant
	HDG_TOL = 10 # heading tolerance +/- degrees
	MIN_ANG_Z = 0.5 # limit rad/s values sent to Turtlebot3
	MAX_ANG_Z = 1.5 # limit rad/s values sent to Turtlebot3

	"""Class that controls subsystems on Turtlebot3"""
	def __init__(self):
		
		self.curr_yaw = 0
		self.goal_yaw = 0
		self.cmd = Twist()
		self.cmd.linear.x = 0.0
		self.cmd.angular.z = 0.0
		self.turning = False
		rospy.Subscriber('mouse_info', MouseController, self.callback_mouseControl)
		rospy.Subscriber('imu', Imu, self.callback_imu)
		rospy.Subscriber('imu', Imu, self.callback_controller)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.ctrl_c = False
		rospy.on_shutdown(self.shutdownhook)


	def callback_mouseControl(self, mouseInfo):
		#TODO 4 Scale xPos from -1 to 1 to -.5 to .5
		#How would you scale? just 0.5*?
		self.cmd = Twist()
		#TODO 5 set angular z in Twist message to the scaled value in the appropriate direction
		self.xPos = (((mouseInfo.xPos - -1) * (.5 - -.5)) / (1 - -1)) + -.5
		self.cmd.angular.z = self.xPos
		#TODO 6 Scale yPos from -1 to 1 to -.5 to .5
		self.yPos = (((mouseInfo.yPos - -1) * (.5 - -.5)) / (1 - -1)) + -.5
		#TODO 7 set linear x in Twist message to the scaled value in the appropriate direction
		self.cmd.linear.x = self.yPos
		#TODO 8 publish the Twist message
		self.pub.publish(self.cmd)
		
	def convert_yaw (self, yaw):
		return 360 + yaw if yaw < 0 else yaw   
		
	def callback_imu(self, imu):
		if not self.ctrl_c:
			# TODO: create a quaternion using the x, y, z, and w values
			# from the correct imu message
			q = Quaternion(imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z)

			# TODO: convert the quaternion to euler in degrees
			self.e = q.to_euler(degrees=True)
			
			# TODO: get the yaw component of the euler
			yaw = self.e[2]

			# convert yaw from -180 to 180 to 0 to 360
			self.curr_yaw = self.convert_yaw(yaw)
			#print("Current heading is %f degrees." % (yaw))
				  
	def callback_controller(self, event):
	# local variables do not need the self
		yaw_err = 0 
		ang_z = 0
	# not turning, so get user input
		if self.turning == False:
			#read from user and set value to instance variable, self.goal_yaw
			#input("Input l or r to turn 90 deg")
			print("Input l or r to turn 90 deg")
			#Here is where I got help from C/
			inputletter = input()
		
			# check input and determine goal yaw
			if inputletter == "l":
			#set goal yaw to curr yaw plus/minus 90
				self.goal_yaw = self.curr_yaw + 90
			#turning equals True
				self.turning = True
			elif inputletter == "r":
			#set goal yaw to curr yaw plus/minus 90
				self.goal_yaw = self.curr_yaw - 90
				self.turning = True
			else:
				print("ERROR: INVALID INPUTS")
			
	# check bounds
			if self.goal_yaw < 0:
				self.goal_yaw = self.goal_yaw + 360
			elif self.goal_yaw > 360:
				self.goal_yaw = self.goal_yaw - 360
	
	# turn until goal is reached
		elif self.turning == True:
			yaw_err = self.curr_yaw - self.goal_yaw
			# determine if robot should turn clockwise or counterclockwise
			if yaw_err > 180:
				yaw_err = yaw_err - 360
			elif yaw_err < -180:
				yaw_err = yaw_err + 360
			# proportional controller that turns the robot until goal 
			# yaw is reached
			ang_z = self.K_HDG * yaw_err

			if ang_z < self.MIN_ANG_Z: 
				ang_z = self.MIN_ANG_Z		
			elif ang_z > -self.MIN_ANG_Z:
				ang_z = -self.MIN_ANG_Z	
			elif ang_z > self.MAX_ANG_Z: 
				ang_z = self.MAX_ANG_Z	
			elif ang_z < -self.MAX_ANG_Z: 
				ang_z = -self.MAX_ANG_Z	
			
			# check goal orientation
			if abs(yaw_err) < self.HDG_TOL:
				self.turning = False
				ang_z = 0

   		# set twist message and publish
		self.cmd.linear.x = 0
		self.cmd.angular.z = ang_z
		self.pub.publish(self.cmd)
	
	def shutdownhook(self):
		print("Controller exiting. Halting robot.")
		self.ctrl_c = True
		#TODO 9 force the linear x and angular z commands to 0 before halting
		self.cmd.linear.x = 0.0
		self.cmd.angular.z = 0.0
		self.pub.publish(self.cmd)

if __name__ == '__main__':
	rospy.init_node('controller')
	c = Controller()
	rospy.spin()