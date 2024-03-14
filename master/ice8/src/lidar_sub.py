#!/usr/bin/env python3
#Looking at node ld08_driver and node hls_lfcd_lds_driver
import rospy, math
# TODO: import correct message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180./math.pi)
# convert LaserScan degree from -180 - 180 degs to 0 - 360 degs
DEG_CONV = lambda deg: deg + 360 if deg < 0 else deg

class LIDAR:    
    """Class to read lidar data from the Turtlebot3 LIDAR"""
    def __init__(self):
        #examples from lab  2
        #rospy.Subscriber('imu', Imu, self.callback_controller)
		#self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        # TODO: create a subscriber to the scan topic published by the lidar launch file
        rospy.Subscriber('scan', LaserScan, self.callback_lidar)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

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
                if deg > 345 or deg < 15:
                    cr30 += 1
                    cr30tot += rng

            # TODO: ensure you don't divide by 0 and print average off the nose
            if cr30 != 0:
                cr30avg = cr30tot / cr30
                print("The average distance off the nose is", cr30avg)
            elif cr30 == 0:
                print("ERROR: Divide by Zero Issue")
                

    def shutdownhook(self):
        print("Shutting down lidar subscriber")
        self.ctrl_c = True

if __name__ == '__main__':
    rospy.init_node('lidar_sub')
    LIDAR()
    rospy.spin()