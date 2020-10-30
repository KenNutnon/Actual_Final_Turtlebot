#!/usr/bin/env python
# BEGIN ALL

#new perspective change
#some formating

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class GoForward():
  def __init__(self):


    # self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callbac

    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                  Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)

    self.twist = Twist()
    self.cmd_vel_pub.publish(self.twist)
 
  # def odom_callback(self, odom_data):
  #     self.current_pose_x = odom_data.pose.pose.position.x
  #     self.current_pose_y = odom_data.pose.pose.position.y
  #     print("x:", int(self.current_pose_x*100)*0.01)
  #     # print("y:", int(self.current_pose_y*100)*0.01)

                      
      
  def image_callback(self, msg):

    for i in range(67000):
      self.twist.linear.x = 0.4
      self.twist.angular.z = 0
      self.cmd_vel_pub.publish(self.twist)

    for i in range(55000):
      self.twist.linear.x = 0.4
      self.twist.angular.z = -0.6
      self.cmd_vel_pub.publish(self.twist)


    self.twist.linear.x = 0
    self.twist.angular.z = 0
    self.cmd_vel_pub.publish(self.twist)
    rospy.sleep(5)



 
rospy.init_node('GoForward')
goforward = GoForward()
rospy.spin()
# END ALL