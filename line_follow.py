#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
from geometry_msgs.msg import Twist
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from statistics import mean 
 
class image_converter:
	def __init__(self):
		self.image_pub = rospy.Publisher("image_topic_2",Image)
		self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		(rows,cols,channels) = cv_image.shape

		gray_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY) 
		threshold = 100 # all pixels under this threshold value will be considered
		row_to_use = rows-100
		center_x=0
		under_thresh = [] 
		count = 0
		summ = 0
		for i in range(len(gray_img[row_to_use])):
		    if(gray_img[row_to_use][i] < threshold):
		        under_thresh.append(i)
		        summ += i
		        count += 1
		#print(under_thresh) 
		if not under_thresh: 
		    #if list is empty use the previous value
			under_thresh = [center_x, center_x]
			count = 1
		center_x = int(summ/count)

		# Center coordinates
		center_coordinates = (center_x, row_to_use) # change this depending on the frame

		
		#if cols > 60 and rows > 60 :
		cv2.circle(cv_image, (center_x,row_to_use), 10, 255)

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)
		move = Twist()

		if(abs(center_x-cols/2)<=50):
			move.linear.x = 0.5
			move.angular.z = 0
		else:
			if((center_x-cols/2) >0):
				move.linear.x = 0
				move.angular.z = -0.5
			else:
				move.linear.x = 0
				move.angular.z = 0.5


		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
			self.move_pub.publish(move)
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