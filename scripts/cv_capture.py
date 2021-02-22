#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter():
  def __init__(self):
    self.image_pub = rospy.Publisher("scan",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image/compressed",CompressedImage,self.callback, queue_size=1)
    # self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback, queue_size=1)
  
  def callback(self,data):
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
    except CvBridgeError as e:
      print(e)

    edges = cv2.Canny(cv_image,100,200)

    cv2.imshow("Image window", edges)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)



def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)