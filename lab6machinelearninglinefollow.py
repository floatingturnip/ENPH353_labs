
import cv2
import gym
import math
import rospy
import roslaunch
import time
import numpy as np
from statistics import mean 
import bisect


from cv_bridge import CvBridge, CvBridgeError
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import Image
from time import sleep

from gym.utils import seeding


class Gazebo_Lab06_Env(gazebo_env.GazeboEnv):

	def __init__(self):
        # Launch the simulation with the given launchfile name
		LAUNCH_FILE = '/home/fizzer/enph353_gym-gazebo/gym_gazebo/envs/ros_ws/src/enph353_lab06/launch/lab06_world.launch'
		gazebo_env.GazeboEnv.__init__(self, LAUNCH_FILE)
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

		self.action_space = spaces.Discrete(3)  # F,L,R
		self.reward_range = (-np.inf, np.inf)
		self.episode_history = []

		self._seed()

		self.bridge = CvBridge()
		self.timeout = 0  # Used to keep track of images with no line detected
		#self.laststate = 0

		self.lower_blue = np.array([97,  0,   0])
		self.upper_blue = np.array([150, 255, 255])

	def process_image(self, data):
		'''
		@brief Coverts data into a opencv image and displays it
		@param data : Image data from ROS

		@retval (state, done)
		'''
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imshow("Image window", cv_image)


		state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		done = False

        # TODO: Analyze the cv_image and compute the state array and
        # episode termination condition.
        #
        # The state array is a list of 10 elements indicating where in the
        # image the line is:
        # i.e.
        #    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0] indicates line is on the left
        #    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0] indicates line is in the center
        #
        # The episode termination condition should be triggered when the line
        # is not detected for more than 30 frames. In this case set the done
        # variable to True.
        #
        # You can use the self.timeout variable to keep track of which frames
        # have no line detected.

		gray_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY) 

		(rows,cols,channels) = cv_image.shape
		#print(rows)

		#print(gray_img)
		threshold = 125 # all pixels under this threshold value will be considered
		row_to_use = rows-10
		#print(gray_img[row_to_use])
		horizontal_len = len(gray_img[row_to_use])
		#print(horizontal_len)
		split_size = int(horizontal_len/10)

		range_dict = {	0: 0,
						split_size: 1,
						2*split_size: 2,
						3*split_size: 3,
						4*split_size: 4,
						5*split_size: 5,
						6*split_size: 6,
						7*split_size: 7,
						8*split_size: 8,
						9*split_size: 9,
						}
		sorted_keys = sorted(range_dict.keys())
		#center_x = 0
		under_thresh = [] # 
		for i in range(len(gray_img[row_to_use])):
			if(gray_img[row_to_use][i] < threshold):
				under_thresh.append(i)
		#print(under_thresh) 
		if len(under_thresh) < 2: 
		#if list is empty use the previous value
			#under_thresh = [self.laststate, self.laststate]
			self.timeout += 1
		else:
			self.timeout = 0
			center_x = int(mean(under_thresh))
		#self.laststate = center_x
		#print(center_x)
			insertion_point = bisect.bisect_left(sorted_keys,center_x)
			

			# adjust, as bisect returns not exactly what we want
			if insertion_point==len(sorted_keys) or sorted_keys[insertion_point]!=center_x:
				insertion_point-=1
			state_index = range_dict[sorted_keys[insertion_point]]
			#print(insertion_point,center_x,range_dict[sorted_keys[insertion_point]])
			#state = [0,0,0,0,0,0,0,0,0,0]
			state[state_index] = 1
		print(state)
		if self.timeout >= 30:
				done = True
		# Center coordinates
		#center_coordinates = (center_x, row_to_use) # change this depending on the frame

		return state, done

	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def step(self, action):

		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")

		self.episode_history.append(action)

		vel_cmd = Twist()

		if action == 0:  # FORWARD
			vel_cmd.linear.x = 0.4
			vel_cmd.angular.z = 0.0
		elif action == 1:  # LEFT
			vel_cmd.linear.x = 0.0
			vel_cmd.angular.z = 0.5
		elif action == 2:  # RIGHT
			vel_cmd.linear.x = 0.0
			vel_cmd.angular.z = -0.5

		self.vel_pub.publish(vel_cmd)

		data = None
		while data is None:
			try:
				data = rospy.wait_for_message('/pi_camera/image_raw', Image, timeout=5)
			except:
				pass

		rospy.wait_for_service('/gazebo/pause_physics')
		try:
		    # resp_pause = pause.call()
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")

		state, done = self.process_image(data)

		# Set the rewards for your action
		if not done:
			if action == 0:  # FORWARD
				reward = 5
			elif action == 1:  # LEFT
				reward = 1
			else:
				reward = 1 # RIGHT
		else:
			reward = -200

		return state, reward, done, {}

	def reset(self):

		print("Episode history: {}".format(self.episode_history))
		self.episode_history = []
		print("Resetting simulation...")
		# Resets the state of the environment and returns an initial
		# observation.
		rospy.wait_for_service('/gazebo/reset_simulation')
		try:
		    # reset_proxy.call()
			self.reset_proxy()
		except (rospy.ServiceException) as e:
			print ("/gazebo/reset_simulation service call failed")

		# Unpause simulation to make observation
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
		    # resp_pause = pause.call()
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")

		# read image data
		data = None
		while data is None:
			try:
				data = rospy.wait_for_message('/pi_camera/image_raw', Image, timeout=5)
			except:
				pass

		rospy.wait_for_service('/gazebo/pause_physics')
		try:
		    # resp_pause = pause.call()
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")

		self.timeout = 0
		state, done = self.process_image(data)

		return state
