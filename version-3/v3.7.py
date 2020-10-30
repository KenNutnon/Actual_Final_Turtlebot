#!/usr/bin/env python
# BEGIN ALL

#new perspective change
#some formating

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


mid_white = False
straight = False
right = False
left = False
stop = False
i = 0

class Follower:
  def __init__(self):
    
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()
    
    self.cmd_vel_pub.publish(self.twist)
  

  def image_callback(self, msg):
    global fol_w, mid_white, straight, right, left, stop, i

    self.fol_w = False

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


    pts1_2 = np.float32([[120,190],[255,190],[0,287],[383,287]])
    pts2_2 = np.float32([[0+40,0],[400+40,0],[0+40,400],[400+40,400]])
    M_2 = cv2.getPerspectiveTransform(pts1_2,pts2_2)
    image_2 = cv2.warpPerspective(img,M_2,(img_size[0]+100,img_size[1]+100))#img_size
    
    # masking color (white and yellow) - me
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([ 27, 207, 213])
    upper_yellow = np.array([180, 255, 255])

    lower_white = np.array([100,100,200], dtype= "uint8")
    upper_white = np.array([255,255,255], dtype= "uint8")

    #threshold to get only white (and yellow - me)
    mask_w = cv2.inRange(image, lower_white, upper_white)
    mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask_w2 = cv2.inRange(image_2, lower_white, upper_white)

    #more image crop - me
    h, w, d = image.shape
    search_top = 3*h/4       + 5
    search_bot = 3*h/4 + 20  + 5

    mask_w[0:search_top, 0:w] = 0
    mask_w[search_bot:h, 0:w] = 0

    mask_y[0:search_top, 0:w] = 0
    mask_y[search_bot:h, 0:w] = 0


    search_left = w*2/5
    search_right = w*3/5
    mask_w2[0:h, 0:search_left] = 0
    mask_w2[0:h, search_right:w] = 0
    mask_w2[0:h*1/4, 0:w] = 0

    self.Mw = cv2.moments(mask_w)
    self.My = cv2.moments(mask_y)

    self.Mw2 = cv2.moments(mask_w2)


    #Start Control - me

    if mid_white:

      if straight:

        if i <= 100:
          i += 1
          self.twist.linear.x = 0.4
          self.twist.angular.z = 0
        else:
          mid_white = False
          straight = False
          i = 0

      elif right:

        if i <= 115:
          i += 1
          self.twist.linear.x = 0.4
          self.twist.angular.z = 0
        elif i <= 160:
          i += 1
          self.twist.linear.x = 0.4
          self.twist.angular.z = -0.6
        else:
          mid_white = False
          right = False
          i = 0

      # elif stop:

      #   if i <= 250:
      #     i += 1
      #     self.twist.linear.x = 0
      #     self.twist.angular.z = 0
      #   else:
      #     mid_white = False
      #     stop = False
      #     i = 0

      else:
        if i <= 5:
          i += 1
          self.twist.linear.x = 0.2
          self.twist.angular.z = 0
        else:
          mid_white = False
          straight = False
          i = 0

      
      print(i)
        



    elif self.that_white_line() and self.fol_w == False:

      print("YES")
      mid_white = True

      self.twist.linear.x = 0
      self.twist.angular.z = 0
      self.cmd_vel_pub.publish(self.twist)

      if self.Img_detect(image0, "Straight-sign.png",10):

        straight = True
      
      
      elif self.Img_detect(image0, "Turn-right-sign3.png",15):
        print("right")

        right = True
        

      # elif self.Img_detect(image0, "Stop-sign.png",15):

      #   stop = True
        



    else:

      print("2")

      speed = 0.3
      
      if self.Mw['m00'] > 0 and self.My['m00'] > 0:
        print("2.1")

        err = self.follow_both(image, w)        #BOTH

        self.twist.linear.x = speed
        self.twist.angular.z = (-float(err) / 100)*3

      elif self.My['m00'] > 0:
        print("2.2")

        err = self.follow_yellow(image, w)      #Yellow

        self.twist.linear.x = speed
        self.twist.angular.z = (-float(err) / 100)*3


        # print('err:', err)
        # print('angle:', self.twist.angular.z)

      elif self.Mw['m00'] > 0:
        print("2.3")

        err = self.follow_white(image, w)       #White

        self.twist.linear.x = speed
        self.twist.angular.z = (-float(err) / 100)*3


        # print('err:', err)
        # print('angle:', self.twist.angular.z)

      else:
        print("2.4")

        err = 0
        self.fol_w = False
        
        self.twist.linear.x = 0
        self.twist.angular.z = 0
    
    self.cmd_vel_pub.publish(self.twist)
    # END CONTROL

    if self.fol_w:
      print("YES")

    #cv2.imshow("rgb_yw2", rgb_yw2)
    #cv2.imshow("rgb_yw", rgb_yw)
    # cv2.imshow("mask_y", mask_y)
    cv2.imshow("mask_w", mask_w)
    cv2.imshow("mask_w_2", mask_w2)
    # cv2.imshow("image0", image0)
    cv2.imshow("image", image)

    cv2.waitKey(3)
    
  def follow_white(self, img, width):
    self.fol_w = True

    cx_w = int(self.Mw['m10']/self.Mw['m00'])
    cy_w = int(self.Mw['m01']/self.Mw['m00'])
    cv2.circle(img, (cx_w, cy_w), 20, (0,255,255), 2)

    cx_tw = cx_w + 160
    if cx_tw >= 265:
      cx_tw = cx_w
    self.fol_w = False  

    err = cx_tw - width/2

    # print("cx_tw:", cx_tw)
    return err
  
  def follow_yellow(self, img, width):
    self.fol_w = False

    cx_y = int(self.My['m10']/self.My['m00'])
    cy_y = int(self.My['m01']/self.My['m00'])
    cv2.circle(img, (cx_y, cy_y), 20, (0,255,255), 2)

    cx_ty = cx_y - 160
    err = cx_ty - width/2



    # print("cx_ty:", cx_ty)
    return err

  def follow_both(self, img, width):
    self.fol_w = False

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

  def that_white_line(self):
    if self.Mw2['m00'] > 0:
      return True
    else:
      return False

  def Img_detect(self, cam, sign, Threshold):
      
    # Threshold
    MIN_MATCH_COUNT = Threshold

    # Initiate SIFT detector
    sift=cv2.xfeatures2d.SIFT_create()


    # Create the Flann Matcher object
    FLANN_INDEX_KDITREE=0
    flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
    flann=cv2.FlannBasedMatcher(flannParam,{})


    train_img = cv2.imread(sign,0)  # train image
    kp1,desc1= sift.detectAndCompute(train_img,None) # find the keypoints and descriptors with SIFT
    

    frame = cam
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    kp2, desc2 = sift.detectAndCompute(gray,None)   # find the keypoints and descriptors with SIFT  of the frame captured
    
    matches=flann.knnMatch(desc2,desc1,k=2)


    # store all the good matches as per Lowe's ratio test.
    goodMatch=[]
    for m,n in matches:
      if(m.distance<0.75*n.distance):
        goodMatch.append(m)

    if(len(goodMatch)>MIN_MATCH_COUNT):

      return True

    else:

      return False
    cv2.imshow('result',frame)

  
rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
