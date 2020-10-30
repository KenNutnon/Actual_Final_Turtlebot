#!/usr/bin/env python
# BEGIN ALL

#new perspective change
#some formating

import rospy, cv2, cv_bridge
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    
    self.Right_sub = rospy.Subscriber('Right_sign',
                                      String, self.right)
    # self.Right_sub = rospy.Subscriber('Left_sign',
    #                                   String, self.left)
    self.Right_sub = rospy.Subscriber('Straight_sign',
                                      String, self.straight)

    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()
    
    self.cmd_vel_pub.publish(self.twist)



  
  def right(self, data):
    print("01")
    self.Right = bool(data.data)
  
  # def left(self, data):
  #   print("02")
  #   self.Left = bool(data.data)

  def straight(self, data):
    print("03")
    self.Straight = bool(data)


  def image_callback(self, msg):
    image0 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
   
    #transformation
    img = cv2.resize(image0,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
    #print img.shape
    rows, cols, ch = img.shape
    pts1 = np.float32([[105,200],[275,200],[0,287],[383,287]])
    pts2 = np.float32([[0+40,0],[400+40,0],[0+40,400],[400+40,400]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    img_size = (img.shape[1], img.shape[0])
    image = cv2.warpPerspective(img,M,(img_size[0]+100,img_size[1]+100))#img_size
    
    # masking color (white and yellow) - me
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([ 27, 207, 213])
    upper_yellow = np.array([180, 255, 255])

    lower_white = np.array([100,100,200], dtype= "uint8")
    upper_white = np.array([255,255,255], dtype= "uint8")

    #threshold to get only white (and yellow - me)
    mask_w = cv2.inRange(image, lower_white, upper_white)
    mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)

    h, w, d = image.shape
    search_top = 2*h/4 
    search_bot = 2*h/4 + 20

    mask_w[0:search_top, 0:w] = 0
    mask_w[search_bot:h, 0:w] = 0

    mask_y[0:search_top, 0:w] = 0
    mask_y[search_bot:h, 0:w] = 0

    self.Mw = cv2.moments(mask_w)
    self.My = cv2.moments(mask_y)


    #sign detect

    # Right = self.Img_detect(image0, "Turn-right-sign.png")
    # Left = None
    # Straight = self.Img_detect(image0, "Straight-sign.png")


    # elif self.Img_detect(image0, "Turn-left-sign.png"):

    #   err = self.follow_white(image, w)
    #   self.twist.linear.x = 0.4
    #   self.twist.angular.z = (-float(err) / 100)*2.5


    if self.Straight:
      print("1")

      err = self.follow_white(image, w)
      self.twist.linear.x = 0.4
      self.twist.angular.z = (-float(err) / 100)*2.5
    
    
    elif self.Right:
      print("0")
      err = self.follow_yellow(image, w)
      self.twist.linear.x = 0.4
      self.twist.angular.z = (-float(err) / 100)*2.5

    # elif self.Img_detect(image0, "Stop-sign.png"):
    #   print("2")

    #   self.twist.linear.x = 0
    #   self.twist.angular.z = 0

    elif self.Mw['m00'] > 0 and self.My['m00'] > 0:
      print("3")

      err = self.follow_both(image, w)
      self.twist.linear.x = 0.4
      self.twist.angular.z = (-float(err) / 100)*2.5

    elif self.My['m00'] > 0:
      print("4")

      err = self.follow_yellow(image, w)
      self.twist.linear.x = 0.4
      self.twist.angular.z = (-float(err) / 100)*2.5

    elif self.Mw['m00'] > 0: #or self.Img_detect(image0, "Turn-left-sign.png"):
      print("5")

      err = self.follow_white(image, w)
      self.twist.linear.x = 0.4
      self.twist.angular.z = (-float(err) / 100)*2.5

    else:
      print("6")
      self.twist.linear.x = 0
      self.twist.angular.z = 0
      err = 0
    
    self.cmd_vel_pub.publish(self.twist)

    # END CONTROL

    #cv2.imshow("rgb_yw2", rgb_yw2)
    #cv2.imshow("rgb_yw", rgb_yw)
    # cv2.imshow("mask_y", mask_y)
    # cv2.imshow("mask_w", mask_w)
    cv2.imshow("image0", image0)
    cv2.imshow("image", image)

    cv2.waitKey(3)
    
  def follow_white(self, img, width):
    cx_w = int(self.Mw['m10']/self.Mw['m00'])
    cy_w = int(self.Mw['m01']/self.Mw['m00'])
    cv2.circle(img, (cx_w, cy_w), 20, (0,255,255), 2)

    cx_tw = cx_w + 160
    err = cx_tw - width/2
    return err
  
  def follow_yellow(self, img, width):
    cx_y = int(self.My['m10']/self.My['m00'])
    cy_y = int(self.My['m01']/self.My['m00'])
    cv2.circle(img, (cx_y, cy_y), 20, (0,255,255), 2)

    cx_ty = cx_y - 160
    err = cx_ty - width/2
    return err

  def follow_both(self, img, width):

    cx_w = int(self.Mw['m10']/self.Mw['m00'])
    cy_w = int(self.Mw['m01']/self.Mw['m00'])
    cv2.circle(img, (cx_w, cy_w), 20, (0,0,255), 2)

    cx_y = int(self.My['m10']/self.My['m00'])
    cy_y = int(self.My['m01']/self.My['m00'])
    cv2.circle(img, (cx_y, cy_y), 20, (0,0,255), 2)

    cx_t = (cx_w + cx_y)/2
    cy_t = (cy_w + cy_y)/2

    cv2.circle(img, (cx_t, cy_t), 20, (0,255,0),2)
    err = cx_t - width/2
    return err

  
rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
