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
#import imutils

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("test_image",Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/ir/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") #?
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)
    #gray = cv2.cvtColor(cv_image, cv2.COLOR_MONO2GRAY)

    blurred = cv2.GaussianBlur(cv_image, (17, 17), 0)
    #(T, thresh) = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY_INV)

      # find contours in the mask and initialize the current
    # (x, y) center of the ball
    #cnts = cv2.findContours(blurred.copy(), cv2.RETR_EXTERNAL,
    #  cv2.CHAIN_APPROX_SIMPLE)
    #cnts = imutils.grab_contours(cnts)
    #center = None
    # only proceed if at least one contour was found
    #if len(cnts) > 0:
      # find the largest contour in the mask, then use
      # it to compute the minimum enclosing circle and
      # centroid
    #  c = max(cnts, key=cv2.contourArea)
    #  ((x, y), radius) = cv2.minEnclosingCircle(c)
    #  M = cv2.moments(c)
    #  center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
      # only proceed if the radius meets a minimum size
    #  if radius > 10:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
    #    cv2.circle(frame, (int(x), int(y)), int(radius),
    #      (0, 255, 255), 2)
    #   cv2.circle(frame, center, 5, (0, 0, 255), -1)
    # update the points queue
    #pts.appendleft(center)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(blurred, "mono16"))
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