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
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()
    
    self.cmd_vel_pub.publish(self.twist)
  

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
    mask_w2 = cv2.inRange(image, lower_white, upper_white)


    h, w, d = image.shape
    search_top = 2*h/4 
    search_bot = 2*h/4 + 20

    left = w*2/5
    right = w*3/5
    mask_w2[0:h, 0:left] = 0
    mask_w2[0:h, right:w] = 0
    mask_w2[0:h*1/4, 0:w] = 0

    self.Mw2 = cv2.moments(mask_w2)

    if self.that_white_line():
      print("YES")
    else:
      print("NO")

    self.Img_detect(image0,"Turn-right-sign3.png", 15)

    # self.Img_detect(image0,"Straight-sign.png", 10)

    #cv2.imshow("rgb_yw2", rgb_yw2)
    #cv2.imshow("rgb_yw", rgb_yw)
    # cv2.imshow("mask_y", mask_y)
    # cv2.imshow("mask_w", mask_w)

    cv2.imshow("image", image)
    cv2.imshow("image0", image0)
    cv2.waitKey(3)


  

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

      print("Match Found-")
      print(len(goodMatch),MIN_MATCH_COUNT)

    else:
      print("Not Enough match found-")
      print(len(goodMatch),MIN_MATCH_COUNT)

    cv2.imshow('result',frame)

  def that_white_line(self):
    if self.Mw2['m00'] > 0:
      return True
    else:
      return False
  
rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
