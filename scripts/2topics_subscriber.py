import message_filters
from std_msgs.msg import Int32, Float32
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#define the list of boundaries
boundaries = [
	([17, 15, 100], [50, 56, 200]),
	([86, 31, 4], [220, 88, 50]),
	([25, 146, 190], [62, 174, 250]),
	([103, 86, 65], [145, 133, 128])
]

lowerRedBoundary = np.array([17, 15, 100], dtype = "uint8")
upperRedBoundary = np.array([50, 56, 200], dtype = "uint8")

def callback(distance, image):
    print(distance.ranges[0])
    bridge = CvBridge()
    try:
      cv_image = bridge.compressed_imgmsg_to_cv2(image)
    except CvBridgeError as e:
      print(e)

    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Red color
    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    hsv_red_output = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)

    cv2.imshow("Image window hsv", hsv_red_output)

    mask = cv2.inRange(cv_image, lowerRedBoundary, upperRedBoundary)
    output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

    cv2.imshow("Image window1", cv_image)
    cv2.imshow("Image window", output)
    cv2.waitKey(1)
  

def twotopic_sub():
    rospy.init_node('2topicsub', anonymous=True)
    mode_sub = message_filters.Subscriber('scan', LaserScan)
    penalty_sub = message_filters.Subscriber('/camera/image/compressed', CompressedImage)

    ts = message_filters.ApproximateTimeSynchronizer([mode_sub, penalty_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        twotopic_sub()
    except rospy.ROSInterruptException:
        pass