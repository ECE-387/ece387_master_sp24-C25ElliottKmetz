#!/usr/bin/env python3
import rospy, cv2, dlib, argparse
from cv_bridge import CvBridge, CvBridgeError
from imutils import paths
from std_msgs.msg import Float32, String

# TODO: import usb_cam message type
from sensor_msgs.msg import Image

# construct the argument parser and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-d", "--detector", required=True, help="Path to trained object detector")
# ap.add_argument("-t", "--testing", required=True, help="Path to directory of testing images")
# args = vars(ap.parse_args())

class StopDetector(object):

    #Known Width
    STOP_WIDTH = 13  #cm
    #Known Distance 
    #known_dist = 40  #cm
    #Focal Length
    FOCAL = 1304.6153846153845
    #Subscribe to the number of pixels width topic
    pixels = int()

    def __init__(self, detectorLoc):
        self.ctrl_c = False

        #TODO: create subscriber to usb_cam image topic
        self.sub = rospy.Subscriber('usb_cam/image_raw', Image, self.camera_callback)

        self.pub = rospy.Publisher('/stop_dist', Float32, queue_size=10)

        self.stoppub = rospy.Publisher('/stop_detect', String, queue_size=10)

        self.bridge_object = CvBridge()
        self.detector = dlib.simple_object_detector(detectorLoc)

        rospy.on_shutdown(self.shutdownhook)

    def camera_callback(self, data):
        if not self.ctrl_c:
            #TODO: write code to get ROS image, convert to OpenCV image,
            # apply detector, add boxes to image, and display image
            try:
                self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
            except CvBridgeError as ERROR:
                print(ERROR)
            # loop over the testing images

            # load the image and make predictions
            image = self.cv_image 
            self.boxes = self.detector(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            
            # loop over the bounding boxes and draw them
            for b in self.boxes:
                (x, y, w, h) = (b.left(), b.top(), b.right(), b.bottom())
                cv2.rectangle(image, (x, y), (w, h), (0, 255, 0), 2)
                self.pixels = w
            
            if self.boxes:
                #print("DETECTING BOXES")
                self.stoppub.publish("stop detected")

            # show the image
            #cv2.imshow("Image", image)
            #cv2.waitKey(1)

        #Calculate the focal length
        #F = (P*D)/W
        #focal = (self.pixels*self.known_dist) / self.STOP_WIDTH
        #print(focal)
        #Calculate the distance from the camera
        #D = (W*F) / P
        Dist = (self.STOP_WIDTH*self.FOCAL) / self.pixels
        self.pub.publish(Dist)

    def shutdownhook(self):
        print("Shutting down")
        self.ctrl_c = True
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('stop_detector')
    detector = rospy.get_param("/stop_detector/detector")
    stop_detector = StopDetector(detector)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass