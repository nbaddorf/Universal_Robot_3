#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("test_image",Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/ir/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono16")
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_MONO2GRAY)

    blurred = cv2.GaussianBlur(gray, (17, 17), 0)
    (T, thresh) = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY_INV)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh, "mono16"))
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