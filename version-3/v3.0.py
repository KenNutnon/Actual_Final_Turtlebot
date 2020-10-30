#!/usr/bin/env python
# BEGIN ALL

#new perspective change
#some formating

import rospy, cv2, cv_bridge
import numpy as np
import matplotlib.pyplot as plt
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
    mask_w = cv2.inRange(image, lower_white, upper_white)
    mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)

    h, w, d = image.shape
    search_top = 2*h/4 
    search_bot = 2*h/4 + 20

    mask_w[0:search_top, 0:w] = 0
    mask_w[search_bot:h, 0:w] = 0

    mask_y[0:search_top, 0:w] = 0
    mask_y[search_bot:h, 0:w] = 0

    Mw = cv2.moments(mask_w)
    My = cv2.moments(mask_y)

    if Mw['m00'] > 0 and My['m00'] > 0:

      cx_w = int(Mw['m10']/Mw['m00'])
      cy_w = int(Mw['m01']/Mw['m00'])
      cv2.circle(image, (cx_w, cy_w), 20, (0,0,255), 2)

      cx_y = int(My['m10']/My['m00'])
      cy_y = int(My['m01']/My['m00'])
      cv2.circle(image, (cx_y, cy_y), 20, (0,0,255), 2)

      cx_t = (cx_w + cx_y)/2
      cy_t = (cy_w + cy_y)/2

      cv2.circle(image, (cx_t, cy_t), 20, (0,255,0),2)

      err = cx_t - w/2
      self.twist.linear.x = 0.4  *0
      self.twist.angular.z = (-float(err) / 100)*2.5  *0

    elif My['m00'] > 0:            #and int(Mw['m10']/Mw['m00'])<2/w:

      cx_y = int(My['m10']/My['m00'])
      cy_y = int(My['m01']/My['m00'])
      cv2.circle(image, (cx_y, cy_y), 20, (0,255,255), 2)

      cx_ty = cx_y - 160
      err = cx_ty - w/2
      self.twist.linear.x = 0.4  *0
      self.twist.angular.z = (-float(err) / 100)*2.5  *0

    elif Mw['m00'] > 0:

      cx_w = int(Mw['m10']/Mw['m00'])
      cy_w = int(Mw['m01']/Mw['m00'])
      cv2.circle(image, (cx_w, cy_w), 20, (0,255,255), 2)

      cx_tw = cx_w + 160
      err = cx_tw - w/2
      self.twist.linear.x = 0.4  *0
      self.twist.angular.z = (-float(err) / 100)*2.5  *0

    else:
    	self.twist.linear.x = 0
    	self.twist.angular.z = 0
    	err = 0

    self.cmd_vel_pub.publish(self.twist)

      # END CONTROL

    # Img_detect(image0, "Turn-right-sign2.png")
    Img_detect(image0, "Straight-sign2.png")

    #cv2.imshow("rgb_yw2", rgb_yw2)
    #cv2.imshow("rgb_yw", rgb_yw)
    # cv2.imshow("mask_y", mask_y)

    # cv2.imshow("mask_w", mask_w)
    cv2.imshow("image", image)
    # cv2.imshow("image0", image0)
    cv2.waitKey(3)


def Img_detect(cam, sign):
    
  # Threshold
  MIN_MATCH_COUNT=10

  # Initiate SIFT detector
  sift=cv2.xfeatures2d.SIFT_create()


  # Create the Flann Matcher object
  FLANN_INDEX_KDITREE=0
  flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
  flann=cv2.FlannBasedMatcher(flannParam,{})



  train_img = cv2.imread(sign,0)  # train image
  kp1,desc1= sift.detectAndCompute(train_img,None) # find the keypoints and descriptors with SIFT
  train_img_kp= cv2.drawKeypoints(train_img,kp1,None,(255,0,0),4) # draw keypoints of the train image



  frame = cam
  gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)   # turn the frame captured into grayscale
  kp2, desc2 = sift.detectAndCompute(gray,None)   # find the keypoints and descriptors with SIFT  of the frame captured
  
  # Obtain matches using K-Nearest Neighbor Method
  # the result 'matches' is the number of similar matches found in both images
  matches=flann.knnMatch(desc2,desc1,k=2)


  # store all the good matches as per Lowe's ratio test.
  goodMatch=[]
  for m,n in matches:
    if(m.distance<0.75*n.distance):
      goodMatch.append(m)

  if(len(goodMatch)>MIN_MATCH_COUNT):

    tp=[]  # src_pts
    qp=[]  # dst_pts
    for m in goodMatch:
        tp.append(kp1[m.trainIdx].pt)
        qp.append(kp2[m.queryIdx].pt)
    tp,qp=np.float32((tp,qp))

    H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)


    h,w = train_img.shape
    train_outline= np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
    query_outline = cv2.perspectiveTransform(train_outline,H)

    cv2.polylines(frame,[np.int32(query_outline)],True,(0,255,0),5)
    cv2.putText(frame,'Object Found',(50,50), cv2.FONT_HERSHEY_COMPLEX, 2 ,(0,255,0), 2)


    print("Match Found-")
    print(len(goodMatch),MIN_MATCH_COUNT)

  else:
    print("Not Enough match found-")
    print(len(goodMatch),MIN_MATCH_COUNT)
  cv2.imshow('result',frame)

  

rospy.init_node('follower')
follower = Follower()

cv2.destroyAllWindows()

rospy.spin()
# END ALL
