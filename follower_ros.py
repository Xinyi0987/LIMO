#!/usr/bin/env python

# source : https://github.com/arjunskumar/Line-Follower--ROS
#

#Steps to run this script on LIMO:
#   From host to rosmaster node to limo host node.
#    on the limo
#       1. ifconfig ( to check IP address of the limo ) 
#       2. open cmd terminal ( ctrl alt T) , gedit ~/.bashrc and change the IP address of the host and master to the IP address of the limo
#       3. source ~/.bashrc 
#       4. roslaunch limo_bringup limo_start.launch and roslaunch astra_camera dabai_u3.launch
#    on the laptop 
#       5. ifconfig ( to check ip address of your host laptop)
#       6. open cmd terminal ( ctrl alt T) , gedit ~/.bashrc and change the IP address of the host laptop and master to the IP address of the limo
#       7. source ~/.bashrc 
#       8. rosrun (package name) follower_ros.py  
#
#    Straight from limo jetson nano ( This is faster due to it not needed to communicate with a master node , thus no lag)
#       1. rosrun (package name) follower_ros.py




#    Description : This code defines a function that defines 2 functions that 
#              1. defines the subcriber node and publisher node
#              2. defines the controls according the region of interest to follow




import rospy          #python library for ROS
import cv2            #module import name for opencv-python
import cv_bridge      #converts openCV images and ROS image messages
import numpy          #python library for working with arrays
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class Follower:
  #description : This function defines the subscriber node and publisher node
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    #subscribe to the image_raw topic
    self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
    #publish to cmd_vel                              
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)                              
    self.twist = Twist()

  #Description: This function takes the input image , converts it from RGB to HSV, 
  #             defines the color range to exclude from masking, 
  #             calculates the center to ensure the area of interest to detect is 
  #             maintained in the middle of the output image
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
	#change image color from BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    	# change below lines to map the color you wanted robot to follow
      # Color for robot to follow is HSV white with a
    sensitivity = 8
    lower_white = numpy.array([ 0,  0, 255-sensitivity])
    upper_white = numpy.array([255, sensitivity, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    h, w, d = image.shape
	#defines how much to mask in the image
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    search_middle = 3*w/4

	  #masking top and bottom of image
    #mask[0:search_top,0:w] = 0
    mask[0:search_top, 0:w] = 0
    mask[search_bot:100, 0:w] = 0

	
    M = cv2.moments(mask)
    if M['m00'] > 0:	#if moment value > 0, calculate the middle of the blob
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
	#create a circle which tracks the color of interest (for visualisation purpose)
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # CONTROL starts
	#error checking
      err = cx - w/2
	#linear speed
      self.twist.linear.x = 0.8
	#turning speed
      self.twist.angular.z = -float(err) / 30
	#publishing the cmd_vel
      self.cmd_vel_pub.publish(self.twist)
      # CONTROL ends
    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(2)





if __name__ == '__main__':
  rospy.init_node('follower')
  follower = Follower()
  rospy.spin()
# END ALL
