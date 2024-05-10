#!/usr/bin/env python3
import rospy, math, time
from std_msgs.msg import Float32, String, Int32
from geometry_msgs.msg import Twist
from squaternion import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from threading import Event
from apriltag_ros.msg import AprilTagDetectionArray

# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180./math.pi)
# convert LaserScan degree from -180 - 180 degs to 0 - 360 degs
DEG_CONV = lambda deg: deg + 360 if deg < 0 else deg

class Controller():

	K_HDG = 0.01 # rotation controller constant
	HDG_TOL = 1 # heading tolerance +/- degrees
	MIN_ANG_Z = 0.1 # limit rad/s values sent to Turtlebot3
	MAX_ANG_Z = 0.5
	# MAX_ANG_Z = 1.5 # limit rad/s values sent to Turtlebot3
	DISTANCE = 0.4 # distance from the wall to stop
	K_POS = 100 # proportional constant for slowly stopping as you get closer to the wall
	MIN_LIN_X = 0.05 # limit m/s values sent to Turtlebot3
	MAX_LIN_X = 0.2 # limit m/s values sent to Turtlebot3
	YAW_GAIN = 0.01
	FOCAL = 1304.6153846153845	#Camera focal length
	distance_threshold = 1	#Distance to stop in meters
	april_id = Int32
	april_dist = Float32
	state = String
	timer = 0
	INIT_HDG = 5000
	low = 0
	high = 1

	def __init__(self):
		self.ctrl_c = False

		# State initialized to wall following
		self.state = "Wall Following State"
	
		#TODO: Joe why do you have all these variables up here??
		self.outlier_test_left = 0
		self.outlier_test_right = 0
		self.curr_yaw = 0; self.goal_yaw = 0; self.cmd = Twist() 
		self.cmd.linear.x = 0.0; self.cmd.angular.z = 0.0
		self.turning = False; self.avg_dist = 0; self.leftavg_dist = 0; self.rightavg_dist = 0
		
		#########################
		# Intersection
		self.leftavg_disti = 0; self.rightavg_disti = 0
		#########################

		#TODO: subscribe and put them into variables hua
		# Lidar, IMU, and Apriltag Subscriber
		rospy.Subscriber('imu', Imu, self.callback_imu)
		rospy.Subscriber('scan', LaserScan, self.callback_lidar)
		rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.callback_april)
		rospy.Subscriber('stop_dist', Float32, self.callback_stopdist)
		rospy.Subscriber('stop_detect', String, self.callback_stop)

		#Controller publisher to robot
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

		rospy.on_shutdown(self.shutdownhook)

	#TODO: Method to store the LiDAR data
	def callback_lidar(self, scan):
		if not self.ctrl_c:
			degrees = []
			ranges = []
			leftlb = float()
			rightlb = float()
			leftup = float()
			rightup = float()

			# determine how many scans were taken during rotation
			count = len(scan.ranges)

			#initialize the variable cr30 to count the ranges 30 degrees of the nose
			#of the robot
			cr30 = 0; cr30tot = 0; cr90left = 0; cr90lefttot = 0; cr90right = 0; cr90righttot = 0
			
			#####################
			# Intersection state
			cr90lefti = 0; cr90lefttoti = 0; cr90righti = 0; cr90righttoti = 0
			#####################

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

				#Create the left and right angle bound for the left and right direction based upon current orientation and initial orientation

				if self.INIT_HDG < 5000:
					right = 300
					left = 60

					rightlb = right - 5
					rightup = right + 5

					leftlb = left - 5
					leftup = left + 5
					
					###################################
					#intersection state values
					righti = 270
					lefti = 90

					rightlbi = righti - 5
					rightupi = righti + 5

					leftlbi = lefti - 5
					leftupi = lefti + 5

					#Right Direction: rng in the left 90 degrees
					if deg > rightlbi and deg < rightupi:
						cr90lefti += 1
						cr90lefttoti += rng

					#Left Direction: rng in the right 90 degrees
					if deg > leftlbi and deg < leftupi:
						cr90righti += 1
						cr90righttoti += rng
					####################################

					#Front Direction: if degrees is between -15 and 15
					if (deg > 355 or deg < 5):
						cr30 += 1
						cr30tot += rng

					#Right Direction: rng in the left 90 degrees
					if deg > rightlb and deg < rightup:
						cr90left += 1
						cr90lefttot += rng

					#Left Direction: rng in the right 90 degrees
					if deg > leftlb and deg < leftup:
						cr90right += 1
						cr90righttot += rng

			# TODO: ensure you don't divide by 0 and print average off the nose
			if cr30 != 0:
				self.avg_dist = cr30tot / cr30
				
			#elif cr30 == 0:
			
			#TODO: Extra credit decide which direction to go in
			# If events are on the left add them to the leftranges list
			if cr90left != 0:
				self.leftavg_dist = cr90lefttot / cr90left
				self.outlier_test_left = cr90left
				# print(cr90left)

			if cr90right != 0:
				self.rightavg_dist = cr90righttot / cr90right
				self.outlier_test_right = cr90right
				#print(self.rightavg_dist)

			##############################
			# Intersection
			if cr90lefti != 0:
				self.leftavg_disti = cr90lefttoti / cr90lefti
				self.outlier_test_lefti = cr90lefti
				# print(cr90left)

			if cr90righti != 0:
				self.rightavg_disti = cr90righttoti / cr90righti
				self.outlier_test_right = cr90righti
				#print(self.rightavg_dist)

			##############################

	#TODO: Function to ensure the yaw is greater than 360		
	def convert_yaw (self, yaw):
		return 360 + yaw if yaw < 0 else yaw   

	#TODO: Function to store the orientation information from the IMU
	def callback_imu(self, imu):
		if not self.ctrl_c:
			q = Quaternion(imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z)
			self.e = q.to_euler(degrees=True)
			yaw = self.e[2]
			# convert yaw from -180 to 180 to 0 to 360
			self.curr_yaw = self.convert_yaw(yaw)
			#print("Current heading is %f degrees." % (yaw))
   
			#Initialize the first heading taken to be the original heading to use as a reference
			if self.INIT_HDG == 5000:

				#FOR ECE HALLWAY FACING EAST FINAL PROJECT HARD CODE
				self.INIT_HDG = 0

				#FOR A DIRECTION OTHER THAN STRAIGHT DOWN HALLWAY
				self.INIT_HDG = self.curr_yaw

	def callback_stopdist(self, stop_dist):

		self.stop_dist = stop_dist.data/100
	#TODO: Function to change the state to state 1
	def callback_stop(self, stop_detected):

		#Set the state to the stop state
		self.state = "Stop Sign Detection"

	#TODO: Function to store the apriltag id in a variable
	def callback_april(self, data):

		if not self.ctrl_c:

			self.april_id = (4,)
			# loop over the bounding boxes and draw them
			for tag in data.detections:
				self.april_id = tag.id
				self.april_dist = tag.pose.pose.pose.position.z
			print(self.april_id[0])
			#Set the state based upon the april tag
			if self.april_id[0] == 0:
				self.state = "Left Turn State"
			elif self.april_id[0] == 1:
				self.state = "Right Turn State"
			elif self.april_id[0] == 2:
				self.state = "U-turn State"
			elif self.april_id[0] == 3:
				self.state = "360 Turn State"

	#TODO: Execute all of the different states hua
	def state_execute(self):

		#TODO: Execute the state0 controller
		#goal: follow the walls and course correct to center
		if self.state == "Wall Following State" and self.INIT_HDG < 5000:
			
			print("Executing State 0: Wall following")
			
			lin_x = 0.5
			ang_z = float()

			# yaw_err = 0.0
			self.cmd.linear.x = lin_x
			self.pub.publish(self.cmd) 
			self.avg_dist_err = self.rightavg_dist - self.leftavg_dist
			
			############################
			# Intersection
			self.avg_dist_erri = self.rightavg_disti - self.leftavg_disti
			############################

			# # #if there is significant difference between both sides and receiving data from the lidar sensor.
			if abs(self.avg_dist_err) >= 0.15 and 4 > self.rightavg_dist and 4 > self.leftavg_dist and abs(self.avg_dist_err - self.avg_dist_erri) < 2:
				if self.avg_dist_err >= 0.15: # closer on left turning right
					ang_z = self.avg_dist_err*self.YAW_GAIN

				elif self.avg_dist_err <= -0.15:	# close on right, turning left
					ang_z = self.avg_dist_err*self.YAW_GAIN

				elif ang_z > self.MAX_ANG_Z: 
					ang_z = self.MAX_ANG_Z	
				elif ang_z < -self.MAX_ANG_Z: 
					ang_z = -self.MAX_ANG_Z

			#ELIF not detecting the walls continue forward until reaching the front wall threshold.
			elif self.avg_dist_erri == 0 and self.avg_dist > 2:
				lin_x = 0.5
				ang_z = 0

			# ELIF for the intersection case ########################################
			elif 0 < self.avg_dist <= 2 and self.avg_dist_erri == 0: #and self.avg_dist_err != 0:

				self.state = "Intersection State"
			#######################################################################	
			
			else:
				lin_x = 0.3

				#Reset to facing East:
				if 20 < self.curr_yaw < 340:

					yaw_err =  self.INIT_HDG - self.curr_yaw

					# # determine if robot should turn clockwise or counterclockwise
					if self.curr_yaw > 180:
						ang_z = -yaw_err * self.YAW_GAIN * 10
					elif self.curr_yaw <= 180:
						ang_z = yaw_err * self.YAW_GAIN * 10

					#Ensure ang_z is within mins
					# if ang_z < self.MIN_ANG_Z: 
					# 	ang_z = self.MIN_ANG_Z		
					# elif ang_z > -self.MIN_ANG_Z:
					# 	ang_z = -self.MIN_ANG_Z	
					elif ang_z > self.MAX_ANG_Z: 
						ang_z = self.MAX_ANG_Z	
					elif ang_z < -self.MAX_ANG_Z: 
						ang_z = -self.MAX_ANG_Z	

				# If facing correct direction: move forward hua
				else:
					lin_x = 0.1
					ang_z = 0
			print(ang_z)		
			print(lin_x)
			self.cmd.angular.z = ang_z
			self.cmd.linear.x = lin_x
			self.pub.publish(self.cmd) 

#########TODO: Execute the state 1 controller ##########################
		#goal: Stop at 1 meter and then wait 5 seconds
		elif self.state == "Stop Sign Detection":

			print("State 1: Stopping")
			
			#print(self.stop_dist)
			#if within the distance threshold, stop
			if self.stop_dist > self.distance_threshold:

				self.cmd.linear.x = 0.05
				self.cmd.angular.z = 0.0
				self.pub.publish(self.cmd)

			elif self.stop_dist <= self.distance_threshold:
				self.timer += 1

				self.cmd.linear.x = 0.0
				self.cmd.angular.z = 0.0
				self.pub.publish(self.cmd)

			#Return back to state 0 after 5 seconds
			# 5 seconds / 0.05 sec == 100
			if self.timer == 100:
				self.state = "Wall Following State"
				self.timer = 0



#########TODO: Execute the state 2 controller ##################################
		#goal: see apriltag0 and stop within 0.5 meters, turn left 90 degrees then turn back to state 0        
		elif self.state == "Left Turn State":
			
			#Initialize a timer and also use it to know the direction you started in. 
			if self.timer == 0:
				init_yaw = self.curr_yaw
				self.low = init_yaw + 87
				self.high = init_yaw + 90
				if self.low > 360:
					self.low -= 360

				if self.high > 360:
					self.high -= 360

			self.timer += 1

			#If within threshold of apriltag
			if self.april_dist < self.distance_threshold:
				
				print("State 2 Executing: Left Turn")
				lin_x = 0
				self.goal_yaw = self.curr_yaw + 90

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
					
					print(yaw_err)
						# check goal orientation
					if abs(yaw_err) < self.HDG_TOL:
						ang_z = 0  #Was missing this jesus :(
						self.turning = False
						
							
				# set twist message and publish
				self.cmd.linear.x = lin_x
				self.cmd.angular.z = ang_z
				self.pub.publish(self.cmd)

			#Return back to state 0 only after successful completion
			if self.low < self.curr_yaw < self.high and self.timer > 20:
				print("time to complete (sec) :", self.timer*0.05)
				self.state = "Wall Following State"
				self.timer = 0



#########TODO: Right Turn State #########################################
		elif self.state == "Right Turn State":

			#Initialize a timer and also use it to know the direction you started in. 
			if self.timer == 0:
				init_yaw = self.curr_yaw
				self.high = init_yaw - 85		#Swap high and low because we are subtracting now
				self.low = init_yaw - 88
				if self.low < 0:
					self.low += 360

				if self.high < 0:
					self.high += 360

			self.timer += 1

			#If within threshold of apriltag
			if self.april_dist < self.distance_threshold:
				
				print("State 3 Executing: Right Turn")

				lin_x = 0
				ang_z = -0.5

				# set twist message and publish
				self.cmd.linear.x = lin_x
				self.cmd.angular.z = ang_z
				self.pub.publish(self.cmd)

			#Return back to state 0 only after successful completion
			if self.low < self.curr_yaw < self.high:
				print("time to complete (sec) :", 0.05*self.timer)
				self.state = "Wall Following State"
				self.timer = 0



#########TODO: State 4 U-Turn behaviour###################################################
		elif self.state == "U-turn State":

			
			#Initialize a timer and also use it to know the direction you started in. 
			if self.timer == 0:
				init_yaw = self.curr_yaw
				self.high = init_yaw - 175		#Swap high and low because we are subtracting now
				self.low = init_yaw - 178
				if self.low < 0:
					self.low += 360

				if self.high < 0:
					self.high += 360

			self.timer += 1
			print(self.april_dist)
			#If within threshold of apriltag
			if self.april_dist < self.distance_threshold:
				
				print("State 4 Executing: U Turn")

				lin_x = 0
				ang_z = -0.5

				# set twist message and publish
				self.cmd.linear.x = lin_x
				self.cmd.angular.z = ang_z
				self.pub.publish(self.cmd)

			#Return back to state 0 only after successful completion
			if self.low < self.curr_yaw < self.high:
				print("time to complete (sec) :", 0.05*self.timer)
				self.state = "Wall Following State"
				self.timer = 0


#########TODO: Execute State 5: 360 turn hua ###############################################
		elif self.state == "360 Turn State":

			#Initialize a timer and also use it to know the direction you started in. 
			if self.timer == 0:
				init_yaw = self.curr_yaw
				self.high = init_yaw - 357		#Swap high and low because we are subtracting now
				self.low = init_yaw - 363
				if self.low < 0:
					self.low += 360

				if self.high < 0:
					self.high += 360

			self.timer += 1

			#If within threshold of apriltag
			if self.april_dist < self.distance_threshold:
				
				print("State 5 Executing: 360")
						
				lin_x = 0
				ang_z = -0.5
				# set twist message and publish
				self.cmd.linear.x = lin_x
				self.cmd.angular.z = ang_z
				self.pub.publish(self.cmd)

			#Return back to state 0 only after successful completion and 50 seconds have passed
			if self.low < self.curr_yaw < self.high and self.timer > 60:
				print("time to complete (sec) :", 0.05*self.timer)
				self.timer = 0
				self.state = "Shutdown State"

		
#########TODO: Execute State 6 T-section stopper ##############################################
		elif self.state == "Shutdown State":
			
			print("SHUTDOWN STATE")
			#Stop and shutdown
			lin_x = 0.0
			ang_z = 0.0
			self.cmd.linear.x = lin_x
			self.cmd.angular.z = ang_z
			self.pub.publish(self.cmd) 

			rospy.signal_shutdown("Objective Completed: Course Complete!")

	def shutdownhook(self):
		print("Shutting down")
		self.ctrl_c = True

if __name__ == '__main__':
	rospy.init_node('controller')
	c = Controller()

	#Define the refresh rate
	rate = rospy.Rate(20) # 20 Hz or occuring every 50 ms

	while not rospy.is_shutdown():
		c.state_execute()

		rate.sleep()
	
	rospy.spin()