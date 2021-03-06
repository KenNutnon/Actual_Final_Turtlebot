#!/usr/bin/env python
# BEGIN ALL

#new perspective change
#some formating

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()

    rospy.on_shutdown(self.shutdown)

    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()
    
    self.cmd_vel_pub.publish(self.twist)

    rospy.sleep(0.2)

    while True:
      self.wakin()
      if rospy.is_shutdown():
        break

  def image_callback(self, msg):
    self.image0 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
   
    #transformation
    img = cv2.resize(self.image0,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
    #print img.shape
    rows, cols, ch = img.shape
    pts1 = np.float32([[105,200],[275,200],[0,287],[383,287]])
    pts2 = np.float32([[0+40,0],[400+40,0],[0+40,400],[400+40,400]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    img_size = (img.shape[1], img.shape[0])
    self.image = cv2.warpPerspective(img,M,(img_size[0]+100,img_size[1]+100))#img_size
    
    # masking color (white and yellow) - me
    hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([ 27, 207, 213])
    upper_yellow = np.array([180, 255, 255])

    lower_white = np.array([100,100,200], dtype= "uint8")
    upper_white = np.array([255,255,255], dtype= "uint8")

    #threshold to get only white (and yellow - me)
    self.mask_w = cv2.inRange(self.image, lower_white, upper_white)
    self.mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)

    self.mask_w2 = cv2.inRange(self.image, lower_white, upper_white)






    #cv2.imshow("rgb_yw2", rgb_yw2)
    #cv2.imshow("rgb_yw", rgb_yw)
    # cv2.imshow("mask_y", mask_y)
    cv2.imshow("mask_w", self.mask_w)
    cv2.imshow("mask_w2", self.mask_w2)
    # cv2.imshow("image0", self.image0)
    cv2.imshow("image", self.image)

    cv2.waitKey(3)
    
  def follow_white(self, img):
    cx_w = int(self.Mw['m10']/self.Mw['m00'])
    cy_w = int(self.Mw['m01']/self.Mw['m00'])
    cv2.circle(img, (cx_w, cy_w), 20, (0,255,255), 2)

    cx_tw = cx_w + 160
    if cx_tw >= 260:
      cx_tw = cx_w

    err = cx_tw - self.w/2


    # print("cx_tw:", cx_tw)
    return err
  
  def follow_yellow(self, img):
    cx_y = int(self.My['m10']/self.My['m00'])
    cy_y = int(self.My['m01']/self.My['m00'])
    cv2.circle(img, (cx_y, cy_y), 20, (0,255,255), 2)

    cx_ty = cx_y - 160
    err = cx_ty - self.w/2



    # print("cx_ty:", cx_ty)
    return err

  def follow_both(self, img):

    cx_w = int(self.Mw['m10']/self.Mw['m00'])
    cy_w = int(self.Mw['m01']/self.Mw['m00'])
    cv2.circle(img, (cx_w, cy_w), 20, (0,0,255), 2)

    cx_y = int(self.My['m10']/self.My['m00'])
    cy_y = int(self.My['m01']/self.My['m00'])
    cv2.circle(img, (cx_y, cy_y), 20, (0,0,255), 2)

    cx_t = (cx_w + cx_y)/2
    cy_t = (cy_w + cy_y)/2

    cv2.circle(img, (cx_t, cy_t), 20, (0,255,0),2)
    err = cx_t - self.w/2
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


  def shutdown(self):
    rospy.loginfo("Stop TurtleBot")
    self.cmd_vel_pub.publish(Twist())
    rospy.sleep(1)
  
  
  def wakin(self):
    global fol_w

    #more image crop - me
    h, self.w, d = self.image.shape
    w = self.w
    search_top = 3*h/4       + 5
    search_bot = 3*h/4 + 20  + 5

    self.mask_w[0:search_top, 0:w] = 0
    self.mask_w[search_bot:h, 0:w] = 0

    self.mask_y[0:search_top, 0:w] = 0
    self.mask_y[search_bot:h, 0:w] = 0


    left = w*2/5
    right = w*3/5
    self.mask_w2[0:h, 0:left] = 0
    self.mask_w2[0:h, right:w] = 0
    self.mask_w2[0:h/2, 0:w] = 0
    self.mask_w2[0:h/2, 0:w] = 0

    self.Mw = cv2.moments(self.mask_w)
    self.My = cv2.moments(self.mask_y)

    self.Mw2 = cv2.moments(self.mask_w2)


    #Start Control - me

    if self.that_white_line() and fol_w == False:

      print("1")

      self.twist.linear.x = 0
      self.twist.angular.z = 0
      self.cmd_vel_pub.publish(self.twist)

      if self.Img_detect(self.image0, "Straight-sign.png",10):
        print("1.1")

        for i in range(50000):
          self.twist.linear.x = 0.4
          self.twist.angular.z = 0
          self.cmd_vel_pub.publish(self.twist)
      
      
      elif self.Img_detect(self.image0, "Turn-right-sign3.png",15):
        print("1.2")

        for i in range(67000):
          self.twist.linear.x = 0.4
          self.twist.angular.z = 0
          self.cmd_vel_pub.publish(self.twist)

        for i in range(55000):
          self.twist.linear.x = 0.4
          self.twist.angular.z = -0.6
          self.cmd_vel_pub.publish(self.twist)

      elif self.Img_detect(self.image0, "Stop-sign.png",15):

        self.twist.linear.x = 0
        self.twist.angular.z = 0

      else:
        
        for i in range(5000):
          self.twist.linear.x = 0.4
          self.twist.angular.z =0
          self.cmd_vel_pub.publish(self.twist)



    else:

      print("2")

      speed = 0.4
      
      if self.Mw['m00'] > 0 and self.My['m00'] > 0:
        print("2.1")

        err = self.follow_both(self.image)        #BOTH
        fol_w = False

        self.twist.linear.x = speed
        self.twist.angular.z = (-float(err) / 100)*3

      elif self.My['m00'] > 0:
        print("2.2")

        err = self.follow_yellow(self.image)      #Yellow
        fol_w = False

        self.twist.linear.x = speed
        self.twist.angular.z = (-float(err) / 100)*3


        # print('err:', err)
        # print('angle:', self.twist.angular.z)

      elif self.Mw['m00'] > 0:
        print("2.3")

        err = self.follow_white(self.image)       #White
        fol_w = True

        self.twist.linear.x = speed
        self.twist.angular.z = (-float(err) / 100)*3


        # print('err:', err)
        # print('angle:', self.twist.angular.z)

      else:
        print("2.4")

        err = 0
        fol_w = False
        
        self.twist.linear.x = 0
        self.twist.angular.z = 0
    
    self.cmd_vel_pub.publish(self.twist)
    # END CONTROL

  
rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
