#!/usr/bin/env python3
import rospy, math
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from squaternion import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from threading import Event

# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180./math.pi)
# convert LaserScan degree from -180 - 180 degs to 0 - 360 degs
DEG_CONV = lambda deg: deg + 360 if deg < 0 else deg

class Controller():
    K_HDG = 0.001 # rotation controller constant
    HDG_TOL = 3 # heading tolerance +/- degrees
    MIN_ANG_Z = 0.5 # limit rad/s values sent to Turtlebot3
    MAX_ANG_Z = 1.5 # limit rad/s values sent to Turtlebot3
    DISTANCE = 0.4 # distance from the wall to stop
    K_POS = 100 # proportional constant for slowly stopping as you get closer to the wall
    MIN_LIN_X = 0.05 # limit m/s values sent to Turtlebot3
    MAX_LIN_X = 0.2 # limit m/s values sent to Turtlebot3
    YAW_GAIN = 5
    INIT_HDG = 5000

    def __init__(self):
        self.ctrl_c = False
        # State Bools
        self.state0 = True; self.state1 = False; self.state2 = False; self.state3 = False
        self.state4 = False; self.state5 = False; self.state6 = False
        # Action complete bool
        self.action = False
        self.ang_update = False
        self.dist_corrected = False
        # State Pubs and Subs
        # self.stop_sub = rospy.Subscriber('stop_detected', Float32, self.stop_callback)
        # self.april_sub = rospy.Subscriber('april_id_dist', Float32, self.april_callback)
        # self.intersect_sub = rospy.Subscriber('intersect_detected', Float32, self.intersect_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        # Lidar and IMU Sub
        self.outlier_test_left = 0
        self.outlier_test_right = 0
        self.curr_yaw = 0; self.goal_yaw = 0; self.cmd = Twist() 
        self.cmd.linear.x = 0.0; self.cmd.angular.z = 0.0
        self.turning = False; self.avg_dist = 0; self.leftavg_dist = 0; self.rightavg_dist = 0
        rospy.Subscriber('imu', Imu, self.callback_imu)
        rospy.Subscriber('scan', LaserScan, self.callback_lidar)
        rospy.Subscriber('imu', Imu, self.callback_controller)
        #rospy.Subscriber('stop_dist', Float32, self.callback_controller)

        # Movement varibles
        
        # States
        # if self.state0 == True:
        #     lin_x = 2
        #     self.avg_dist_err = self.rightavg_dist - self.leftavg_dist
        #     if 2 > self.avg_dist_err > 0:
        #         self.goal_yaw = self.curr_yaw + self.avg_dist_err*self.YAW_GAIN
        #     elif -2 < self.avg_dist_err < 0:
        #         self.goal_yaw = self.curr_yaw - self.avg_dist_err*self.YAW_GAIN
		# # check bounds
        #     if self.goal_yaw < 0:
        #         self.goal_yaw = self.goal_yaw + 360
        #     elif self.goal_yaw > 360:
        #         self.goal_yaw = self.goal_yaw - 360
        #     self.turning = True
		# # turn until goal is reached
        # if self.turning == True:
        #     yaw_err = self.curr_yaw - self.goal_yaw
		# 	# determine if robot should turn clockwise or counterclockwise
        #     if yaw_err > 180:
        #         yaw_err = yaw_err - 360
        #     elif yaw_err < -180:
        #         yaw_err = yaw_err + 360
				
		# 		# proportional controller that turns the robot until goal 
		# 		# yaw is reached
        #     ang_z = self.K_HDG * yaw_err

        #     if ang_z < self.MIN_ANG_Z: 
        #         ang_z = self.MIN_ANG_Z		
        #     elif ang_z > -self.MIN_ANG_Z:
        #         ang_z = -self.MIN_ANG_Z	
        #     elif ang_z > self.MAX_ANG_Z: 
        #         ang_z = self.MAX_ANG_Z	
        #     elif ang_z < -self.MAX_ANG_Z: 
        #         ang_z = -self.MAX_ANG_Z	
			
		# 		# check goal orientation
        #     if abs(yaw_err) < self.HDG_TOL:
        #         ang_z = 0
            
        #     self.cmd.linear.x = lin_x
        #     self.cmd.angular.z = ang_z
        #     self.cmd_pub.publish(self.cmd)
        # def state_controller(self, scan, imu, stop_dist, april_id):


        if self.state1 == True:
            #TODO: Stop
            self.cmd.linear = 0
            self.cmd.angular = 0
            self.cmd_pub.publish(self.cmd)

            #TODO:         
        # if self.state2 == True:
        #     self.action = True
        #     self.cmd.linear = 0
        #     self.cmd.angular = 0
        #     self.cmd_pub.publish(self.cmd)
        #     rospy.sleep(5) #figure out a better way to do this, this will not work everytime
        #     self.action = False
            
        # if self.state3 == True:
        #     self.action = True
        #     self.cmd.linear = 0
        #     self.cmd.angular = 0
        #     self.cmd_pub.publish(self.cmd)

        # if self.state4 == True:
        #     self.cmd.linear = 0
        #     self.cmd.angular = 0
        #     self.cmd_pub.publish(self.cmd)

        rospy.on_shutdown(self.shutdownhook)

    def callback_lidar(self, scan):
        if not self.ctrl_c:
            degrees = []
            ranges = []

            # determine how many scans were taken during rotation
            count = len(scan.ranges)

            #initialize the variable cr30 to count the ranges 30 degrees of the nose
            #of the robot
            cr30 = 0; cr30tot = 0; cr90left = 0; cr90lefttot = 0; cr90right = 0; cr90righttot = 0
			
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
                    right = 270
                    left = 90
                    rightlb = right - 5 + (self.INIT_HDG - self.curr_yaw)
                    rightup = right + 5 + (self.INIT_HDG - self.curr_yaw)

                    leftlb = left - 5 + (self.INIT_HDG - self.curr_yaw)
                    leftup = left + 5 + (self.INIT_HDG - self.curr_yaw)

                    if rightlb > 180:
                        rightlb = rightlb - 360
                    elif rightlb < -180:
                        rightlb = rightlb + 360

                    if rightup > 180:
                        rightup = rightup - 360
                    elif rightup < -180:
                        rightup = rightup + 360

                    if leftup > 180:
                        leftup = leftup - 360
                    elif leftup < -180:
                        lefttup = lefttup + 360

                    if leftlb > 180:
                        leftlb = leftlb - 360
                    elif leftlb < -180:
                        leftlb = leftlb + 360
                                
                # if self.INIT_HDG < 5000:
                #     right = self.INIT_HDG + 270
                #     left = self.INIT_HDG + 90
                #     rightlb = right - 5 - (self.INIT_HDG - self.curr_yaw)
                #     rightup = right + 5 - (self.INIT_HDG - self.curr_yaw)

                #     leftlb = left - 5 - (self.INIT_HDG - self.curr_yaw)
                #     leftup = left + 5 - (self.INIT_HDG - self.curr_yaw)

                #Front Direction: if degrees is between -15 and 15
                if (deg > 345 or deg < 15):
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
                self.got_avg = True
				#print(self.avg_dist)
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
                
                
    def convert_yaw (self, yaw):
        return 360 + yaw if yaw < 0 else yaw   

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
                self.INIT_HDG = self.curr_yaw
   
    def callback_controller(self, event):
        lin_x = 0.1 # self.MAX_LIN_X
        ang_z = 0
        yaw_err = 0
        if self.state0 == True and self.INIT_HDG < 5000:
            self.cmd.linear.x = lin_x
            self.pub.publish(self.cmd) 
            self.avg_dist_err = self.rightavg_dist - self.leftavg_dist
            # print("Right distance error is %f" % (self.rightavg_dist))
            # print("Left distance error is %f" % (self.leftavg_dist))
            print("Average distance error is %f" % (self.avg_dist_err))
            print("Current heading is %f degrees." % (self.curr_yaw))
            # if self.ang_update == True:
            #     ang_z = 0
            #     self.cmd.angular.z = ang_z
            #     self.pub.publish(self.cmd)
            #     self.ang_update = False 
            # if self.dist_corrected == True:
            #     print("DISTANCE CORRECTED")
            #     ang_z = 0
            #     self.cmd.angular.z = ang_z
            #     self.pub.publish(self.cmd) 

            #if there is significant difference between both sides and receiving data from the lidar sensor.
            if abs(self.avg_dist_err) >= 0.15 and self.rightavg_dist > 0 and self.leftavg_dist > 0:
                # self.dist_corrected = False
                if self.turning == False:
                    if self.avg_dist_err >= 0.15: # closer on left
                        self.goal_yaw = self.curr_yaw - abs(self.avg_dist_err)*self.YAW_GAIN
                        
                    elif self.avg_dist_err <= -0.15:
                        self.goal_yaw = self.curr_yaw + abs(self.avg_dist_err)*self.YAW_GAIN
                        
                    if self.goal_yaw < 0:
                        self.goal_yaw = self.goal_yaw + 360

                    elif self.goal_yaw > 360:
                        self.goal_yaw = self.goal_yaw - 360    

                    self.turning = True

            print("Goal heading is %f degrees." % (self.goal_yaw))
            print("Initial Heading was %f degrees." % (self.INIT_HDG))
                
            # print("Yaw Error is %f degrees." % (yaw_err))
            
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

                #Ensure the turn rate never gets very high
                # if abs(ang_z) > 0.10:
                #     ang_z = 0.01

                print("turning at %f" % ang_z)
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
                print("Heading Approved")
                self.turning = False
                # self.ang_update = True
            if self.turning == False:
                self.cmd.linear.x = lin_x
                self.cmd.angular.z = ang_z
                self.pub.publish(self.cmd) 



    # def stop_callback(self,data):
    #     dist = data.distance
    #     if 0.9 < dist < 1.1:
    #         self.state2 = True

    # def april_callback(self,data):
    #     id = data.id

    #     if id == 0:
    #         self.state3 = True
    #     elif id == 1:
    #         self.state4 = True
    #     elif id == 2:
    #         self.state5 = True
    #     elif id == 3:
    #         self.state6 = True

    # def intersect_callback(self,data):
    #     if self.action == False:
    #         self.state1 = True


    def shutdownhook(self):
        print("Shutting down")
        self.ctrl_c = True

if __name__ == '__main__':
    rospy.init_node('controller')
    c = Controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass