#!/usr/bin/env python
# BEGIN ALL


#switch main line from white to yellow

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

global perr, ptime, serr, dt,move
perr = 0
ptime = 0
serr = 0
dt = 0
move = False

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
    global perr, ptime, serr, dt
    image0 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
   
    #transformation
    img = cv2.resize(image0,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
    #print img.shape
    rows, cols, ch = img.shape
    pts1 = numpy.float32([[105,200],[275,200],[8,265],[375,265]])
    pts2 = np.float32([[0+40,0],[400+40,0],[0+40,400],[400+40,400]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    img_size = (img.shape[1], img.shape[0])
    image = cv2.warpPerspective(img,M,(img_size[0]+100,img_size[1]+100))#img_size
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 27, 207, 213])
    upper_yellow = numpy.array([180, 255, 255])

    lower_white = numpy.array([100,100,200], dtype= "uint8")
    upper_white = numpy.array([255,255,255], dtype= "uint8")

    #threshold to get only white
    maskw = cv2.inRange(image, lower_white, upper_white)
    masky = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #remove pixels not in this range
    mask_yw = cv2.bitwise_or(maskw, masky)
    rgb_yw = cv2.bitwise_and(image, image, mask = mask_yw).astype(numpy.uint8)

    rgb_yw = cv2.cvtColor(rgb_yw, cv2.COLOR_RGB2GRAY)
 
    #filter mask
    kernel = numpy.ones((7,7), numpy.uint8)
    opening = cv2.morphologyEx(rgb_yw, cv2.MORPH_OPEN, kernel)
    rgb_yw2 = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
	
    h, w= rgb_yw2.shape
    search_top = 7*h/8+20
    search_bot = 7*h/8 + 800 +20
    rgb_yw2[0:search_top, 0:w] = 0
    rgb_yw2[search_bot:h, 0:w] = 0
    M = cv2.moments(rgb_yw2)
    c_time = rospy.Time.now()
    if M['m00'] > 0:
      cxm = int(M['m10']/M['m00'])
      cym = int(M['m01']/M['m00'])
      
      cx = cxm - 110 #120#143 #CW
      if cxm <= 2*h/8:
      	cx = cxm +(h/2)
      
      cv2.circle(image, (cxm, cym), 20, (255,0,0), 2)
      cv2.circle(image, (cx, cym), 20, (255,0,0), 2)
  
      # BEGIN CONTROL
      err = cx - 4*w/8
      self.twist.linear.x = 0.9*0.5
      dt = rospy.get_time() - ptime
      self.twist.angular.z = ((-float(err) / 100)*2.5 + ((err - perr)/(rospy.get_time() - ptime))*1/50/100)*0.5 #+ (serr*dt)*1/20/100 #1 is best, starting 3 unstable
      serr = err + serr
      perr = err
      ptime = rospy.get_time()

    else:
    	self.twist.linear.x = 0
    	self.twist.angular.z = 0
    	err = 0
     
    self.cmd_vel_pub.publish(self.twist)

      # END CONTROL

    # cv2.imshow("rgb_yw2", rgb_yw2)
    # cv2.imshow("rgb_yw", rgb_yw)
    # cv2.imshow("mask_yw", mask_yw)
    cv2.imshow("image", image)
    cv2.imshow("image0", image0)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL