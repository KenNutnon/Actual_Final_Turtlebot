#!/usr/bin/env python
# BEGIN ALL

import rospy, cv2, cv_bridge
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Img_detect:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.Right_pub = rospy.Publisher('Right_sign',
                                       String, queue_size=1)
    # self.Left_pub = rospy.Publisher('left_sign',
    #                                    String, queue_size=1)
    self.Straight_pub = rospy.Publisher('straight_sign',
                                       String, queue_size=1)


  def image_callback(self, msg):
    self.image0 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')


    
    self.Right_pub.publish(self.Img_detect(self.image0, 'Turn-right-sign2.png'))

    # self.Left_pub.publish(self.Img_detect(self.image0, 'Turn-left-sign2.png'))

    self.Straight_pub.publish(self.Img_detect(self.image0, 'Straight-sign2.png'))


    
  
  def Img_detect(self, cam, sign):
      
    MIN_MATCH_COUNT = 10

    gray = cv2.cvtColor(cam,cv2.COLOR_BGR2GRAY)  
    img1 = gray                               # queryImage
    img2 = cv2.imread(sign,0) # trainImage

    # Initiate SIFT detector
    sift = cv2.xfeatures2d.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    # FLANN parameters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary

    flann = cv2.FlannBasedMatcher(index_params,search_params)

    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    goodMatch=[]
    for m,n in matches:
      if(m.distance<0.75*n.distance):
        goodMatch.append(m)

    if(len(goodMatch)>MIN_MATCH_COUNT):

      return "True"

    else:

      return "False"

rospy.init_node('img_detect')
img_detect = Img_detect()
rospy.spin()
# END ALL