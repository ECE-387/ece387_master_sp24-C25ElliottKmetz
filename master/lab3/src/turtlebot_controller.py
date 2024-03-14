#!/usr/bin/env python3

#turtlebot_controller.py
#For lab1, this will subscribe to mouse_client and publish to cmd_vel
#Will convert messages of type MouseController to Twist
#Deactivates when mouse wheel is scrolled up 
#last modified 9 Feb 2024


import rospy, math
#TODO 2 Import the appropriate message types that we will need
from geometry_msgs.msg import Twist
from lab1.msg import MouseController
from squaternion import Quaternion
from sensor_msgs.msg import Imu
#TODO Import the laser msg from ICE8
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

#TODO Import the lambda functions from ICE8
# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180./math.pi)
# convert LaserScan degree from -180 - 180 degs to 0 - 360 degs
DEG_CONV = lambda deg: deg + 360 if deg < 0 else deg


class Controller:
	#TODO 3 initialize the appropriate Controller class attributes
	K_HDG = 0.1 # rotation controller constant
	HDG_TOL = 3 # heading tolerance +/- degrees
	MIN_ANG_Z = 0.5 # limit rad/s values sent to Turtlebot3
	MAX_ANG_Z = 1.5 # limit rad/s values sent to Turtlebot3
    #TODO initialize new class variables
	DISTANCE = 0.4 # distance from the wall to stop
	K_POS = 100 # proportional constant for slowly stopping as you get closer to the wall
	MIN_LIN_X = 0.05 # limit m/s values sent to Turtlebot3
	MAX_LIN_X = 0.2 # limit m/s values sent to Turtlebot3



	"""Class that controls subsystems on Turtlebot3"""
	def __init__(self):
		
		self.curr_yaw = 0
		self.goal_yaw = 0
		self.cmd = Twist()
		self.cmd.linear.x = 0.0
		self.cmd.angular.z = 0.0
		self.turning = False
		self.avg_dist = 0
		self.leftavg_dist = 0
		self.rightavg_dist = 0
		self.got_avg = False
		rospy.Subscriber('scan', LaserScan, self.callback_lidar)
		rospy.Subscriber('mouse_info', MouseController, self.callback_mouseControl)
		rospy.Subscriber('imu', Imu, self.callback_imu)
		rospy.Subscriber('imu', Imu, self.callback_controller)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.ctrl_c = False
		rospy.on_shutdown(self.shutdownhook)

    #TODO Add the `callback_lidar()` function from ICE8, removing
    #print statements and setting the instance variables, 
	#`self.avg_dist` and `self.got_avg`.
	def callback_lidar(self, scan):
		if not self.ctrl_c:
			degrees = []
			ranges = []

            # determine how many scans were taken during rotation
			count = len(scan.ranges)

            #initialize the variable cr30 to count the ranges 30 degrees of the nose
            #of the robot
			cr30 = 0
			cr30tot = 0
			cr90left = 0
			cr90lefttot = 0
			cr90right = 0
			cr90righttot = 0
			
			for i in range(count):
                # using min angle and incr data determine curr angle, 
                # convert to degrees, convert to 360 scale
				degrees.append(int(DEG_CONV(RAD2DEG(scan.angle_min + scan.angle_increment*i))))
				rng = scan.ranges[i]

                # ensure range values are valid; set to 0 if not
				if rng < scan.range_min or rng > scan.range_max:
					ranges.append(0.0)
				else:
					ranges.append(rng)

            # python way to iterate two lists at once!
			for deg, rng in zip(degrees, ranges):
                # TODO: sum and count the ranges 30 degrees off the nose of the robot
                #rng is curr angle in degrees
                #if degrees is between -15 and 15
				if (deg > 345 or deg < 15):
					cr30 += 1
					cr30tot += rng

				#rng in the left 90 degrees
				if deg > 75 and deg < 105:
					cr90left += 1
					cr90lefttot += rng

				#rng in the right 90 degrees
				if deg > 255 and deg < 295:
					cr90right += 1
					cr90righttot += rng

            # TODO: ensure you don't divide by 0 and print average off the nose
			if cr30 != 0:
				self.avg_dist = cr30tot / cr30
				self.got_avg = True
				#print(self.avg_dist)
			#elif cr30 == 0:
			
			#TODO: Extra credit decide which direction to go in
			#If events are on the left add them to the leftranges list
			if cr90left != 0:
				self.leftavg_dist = cr90lefttot / cr90left
				#print(self.leftavg_dist)

			if cr90right != 0:
				self.rightavg_dist = cr90righttot / cr90right
				#print(self.rightavg_dist)

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
		lin_x = 0
		ang_z = 0
		
	#calculate the distance error
		if self.got_avg == True:
			self.dist_error = self.avg_dist - self.DISTANCE
			if abs(self.dist_error) <= 0.05:
				print(self.dist_error)
		#use dist_error to drive your robot straight 
		#at a proportional rate
			if self.turning == False:
				lin_x = self.K_POS * self.dist_error
		
		#TODO Limit the linear speed of the robot to `MIN_LIN_X` and `MAX_LIN_X`.
				if lin_x < self.MIN_LIN_X:
					lin_x = self.MIN_LIN_X
			#elif lin_x > -self.MIN_LIN_X:
				#lin_x = -self.MIN_LIN_X
				elif lin_x > self.MAX_LIN_X:
					lin_x = self.MAX_LIN_X
			#elif lin_x < self.MAX_LIN_X:
				#lin_x = -self.MAX_LIN_X
	
			if self.dist_error <= 0.05:
				lin_x = 0

	#1. If within `DISTANCE` of a wall, then stop and start 
	#turning (left or right, you decide).  
		if (self.turning == False and lin_x == 0): # or (self.avg_dist <= self.DISTANCE - 0.05 and self.turning == False)
			#Extra credit decision which direction to turn
			if self.leftavg_dist < self.rightavg_dist:
				self.goal_yaw = self.curr_yaw - 90
			elif self.rightavg_dist < self.leftavg_dist:
				self.goal_yaw = self.curr_yaw + 90
			#flipped the directions hua
			
		# check bounds
			if self.goal_yaw < 0:
				self.goal_yaw = self.goal_yaw + 360
			elif self.goal_yaw > 360:
				self.goal_yaw = self.goal_yaw - 360

			
			
			self.turning = True
		# turn until goal is reached
		if self.turning == True:
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
				ang_z = 0  #Was missing this jesus :(
				self.turning = False
					
		# set twist message and publish
		self.cmd.linear.x = lin_x
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