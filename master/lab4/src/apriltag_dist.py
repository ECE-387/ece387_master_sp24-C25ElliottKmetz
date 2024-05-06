#!/usr/bin/env python3
import rospy, cv2, dlib, argparse
from cv_bridge import CvBridge, CvBridgeError
from imutils import paths
from std_msgs.msg import Float32, String
from apriltag_ros.msg import AprilTagDetectionArray

# TODO: import usb_cam message type
from sensor_msgs.msg import Image

class April_Detector():

	FOCAL = 1304.6153846153845
	#Subscribe to the number of pixels width topic
	pixels = int()
	
	def __init__(self):
		self.ctrl_c = False

		#TODO: create subscriber to /tag_detections topic
		self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.april_callback)
		#Publish to the april_id_dist topic our tags
		self.pub = rospy.Publisher('/april_id_dist', Float32, queue_size=10)
		#self.pub2 = rospy.Publisher('/april_dist', Float32, queue_size=10)

		rospy.on_shutdown(self.shutdownhook)

	def april_callback(self, data):
		if not self.ctrl_c:

			# loop over the bounding boxes and draw them
			for tag in data.detections:
				print(tag.id)
				print(100*tag.pose.pose.pose.position.z)

				#self.pub.publish(tag.id)
				#self.pub.publish(100*tag.pose.pose.pose.position.z)

	def shutdownhook(self):
		print("Shutting down")
		self.ctrl_c = True
		cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.init_node('April_Detector')
	april_detector = April_Detector()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		pass