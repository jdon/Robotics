#!/usr/bin/env python
# BEGIN ALL
import math
import numpy as np
from math import sin, cos

import actionlib
import cv2
import cv_bridge
import numpy
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image

import Extras as extra


class Goal:
	def __init__(self, name, lower, upper):
		self.name = name
		self.location = None
		self.running = 0
		self.UpperMask = np.array([upper, 255, 255])
		self.LowerMask = np.array([lower, 40, 40])
		self.found = 0
		self.shownFound = None

	def UpdateMask(self, image):
		self.mask = cv2.inRange(image, self.LowerMask, self.UpperMask)
		return self.mask


def point_pos(x, y, d, rad):
	newx = x + (d - 0.5) * cos(rad)
	newy = y + (d - 0.5) * sin(rad)
	return newx, newy



class self:
	np.set_printoptions(threshold=numpy.nan)
	current_x = 0
	current_y = 0
	current_rad = 0
	resolution = 0.05
	origin = 0
	Header = 0
	width = 216.0
	height = 250.0

	Depthimage = None
	DepthimageMesaures = 0
	bridge = cv_bridge.CvBridge()
	cv2.namedWindow("window", 1)
	twist = Twist()
	Blue = Goal("Blue", 110, 120)
	Yellow = Goal("Yellow", 25, 35)
	Green = Goal("Green", 60, 70)
	Red = Goal("Red", 0, 2)

	foundColour = 0
	destinations = []
	currentGoal = None
	fixedPoints = []
	allFound = 0
	dilatedCostmap = None

	def __init__(self):
		# set ros parameters to make the navigation stack more conserative and avoid walls
		rospy.set_param('/turtlebot/move_base/TrajectoryPlannerROS/occdist_scale', 1.0)
		rospy.set_param('/turtlebot/move_base/TrajectoryPlannerROS/pdist_scale', 5.00)
		rospy.set_param('/turtlebot/move_base/TrajectoryPlannerROS/min_vel_x', 1.00)
		rospy.set_param('/turtlebot/move_base/TrajectoryPlannerROS/max_vel_x', 5.00)

		# set up publishers and subscribers
		self.client = actionlib.SimpleActionClient('turtlebot/move_base', MoveBaseAction)
		self.client.wait_for_server()
		self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=1)
		self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
		self.depth_sub = rospy.Subscriber('/turtlebot/camera/depth/image_raw', Image, self.depth_callback)
		self.costMap = rospy.Subscriber('/turtlebot/move_base/global_costmap/costmap', OccupancyGrid,
										self.costmap_callback)

		self.twist = Twist()
		self.goal = MoveBaseGoal()

		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		rate = rospy.Rate(5.0)

		# use a ros transform to obtain the current location of the robot
		while not rospy.is_shutdown():
			try:
				# look up the transform
				test = tfBuffer.lookup_transform('map', 'turtlebot/base_footprint', rospy.Time())
				pos_x = test.transform.translation.x
				pos_y = test.transform.translation.y
				q_w = test.transform.rotation.w
				q_x = test.transform.rotation.x
				q_y = test.transform.rotation.y
				q_z = test.transform.rotation.z
				roll, pitch, yaw = extra.quaternion_to_euler_angle(q_w, q_x, q_y, q_z)
				self.current_x = pos_x
				self.current_y = pos_y
				self.current_rad = yaw

				# check if within one meter of a goal
				goals = [self.Blue, self.Yellow, self.Red, self.Green]
				for goal in goals:
					if goal.location is not None:
						(x, y) = goal.location
						if not goal.shownFound:
							if (pos_x + 2 > x and x > pos_x - 2):
								if (pos_y + 2 > y and y > pos_y - 2):
									self.FoundGoal(goal)
				rate.sleep()

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rate.sleep()
				continue

	def FoundGoal(self, goal):
		goal.shownFound = True
		print("Found the goal")
		self.rotate()


	def cords_to_costmap(self,x, y):
		newx = int((x - self.origin.position.x)/self.resolution)
		newy = int((y - self.origin.position.y) / self.resolution)
		print("the new x and y are"+str(newx)+ " "+ str(newy))
		# checks to see if the new cords are viable for the navigation stack
		for poy in range(newy-20, newy+20):
			for pox in range(newx-20, newx+20):
				if self.dilatedCostmap[pox][poy] <= 0:
					return pox, poy


	def Map_convert(self, (goal_x, goal_y)):
		# convert from costmap to cords
		goals = [self.Blue, self.Green, self.Yellow, self.Red]
		# not using fixed points so I need to convert them
		if len(self.fixedPoints) == 0:
			x = self.origin.position.x + (goal_x * self.resolution);
			y = self.origin.position.y + (goal_y * self.resolution);
		else:
			print("Going to coloured pole location")
			x = goal_x
			y = goal_y
		print("GoalX:" + str(goal_x) + " X: " + str(x) + " GoalY: " + str(goal_y) + " Y:" + str(y))
		self.moveToGoal(x, y)

	def moveToGoal(self, xGoal, yGoal):
		# sends the goal to the simple action server for the navigation stack
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = xGoal
		goal.target_pose.pose.position.y = yGoal
		goal.target_pose.pose.orientation.w = 1.0
		self.client.send_goal(goal, done_cb=self.donecb, feedback_cb=self.FeedbackCB)

	def donecb(self, state, result):
		# Called when goal is done, gets navigation stack to move to next point
		print("Done callback" + str(result))
		if (state != 2):
			print("Current destinations" + str(self.destinations))
			self.nextDestination()

	def FeedbackCB(self, msg):
		daksdj = "dsd"

	def image_callback(self, msg):
		# finds coloured poles from the camera
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		# Updates the images in the goals to the current image from the camera
		self.Blue.UpdateMask(hsv)
		self.Green.UpdateMask(hsv)
		self.Yellow.UpdateMask(hsv)
		self.Red.UpdateMask(hsv)

		# applies mask to image to show all colours on the same image
		addedMask = self.Blue.mask + self.Yellow.mask + self.Red.mask + self.Green.mask
		res = cv2.bitwise_and(image, image, mask=addedMask)
		average_depth = 1.5
		dres = None
		# setup for loop
		goals = [self.Blue, self.Yellow, self.Red, self.Green]
		h, w, d = image.shape
		for goal in goals:
			# get depth camera info
			if self.Depthimage is not None:
				# apply mask of all goals to depth camera to display to user
				dres = cv2.bitwise_and(self.Depthimage, self.Depthimage, mask=addedMask)
				# get actual depth values for the specfic goal
				depth = cv2.bitwise_and(self.Depthimage, self.DepthimageMesaures, mask=goal.mask)
				# get the average value of values in the mask
				depth[depth == 0] = np.nan
				average_depth = np.nanmean(depth)
				if not np.isnan(average_depth):
					# using depth and current postion, get the x and y to the thing in the mask
					x, y = point_pos(self.current_x, self.current_y, average_depth, self.current_rad)
			# check if the moment is big enough to be a coloured pole
			M = cv2.moments(goal.mask)
			if M['m00'] > 50000.0:  # can see colour
				# lock the this loop so it only looks for one colour
				if self.foundColour == 0 and not goal.found:
					self.foundColour = 1
					goal.running = 1
				# coloured pole is in the vision of the camera, now center the camera
				if goal.running == 1 and self.foundColour == 1 and goal.found == 0:
					cx = int(M['m10'] / M['m00'])
					cy = int(M['m01'] / M['m00'])
					cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
					err = cx - w / 2
					self.twist.angular.z = -float(err) / 200
					self.cmd_vel_pub.publish(self.twist)
					# camera is centered so location of the coloured pole and save it
					if err == 0:
						if 'y' in locals():
							if(y != 0.0):
								print(M['m00'])
								print("found " + goal.name)
								goal.location = self.cords_to_costmap(x, y)
								self.destinations.append(goal.location)
								print(goal.location)
								goal.found = 1
								self.foundColour = 0
								rospy.sleep(1)

				if goal.found:
					# goal is found now keep checking if we are within one meter of it
					if 'average_depth' in locals():
						if average_depth <= 1 and not goal.shownFound:
							# within one meter of the goal
							self.FoundGoal(goal)
							self.foundColour = 0
			else:
				# reset if it sees both but moves to view only one coloured pole
				if goal.found != 0 and self.foundColour == 1:
					goal.running = 0
					self.foundColour = 0
		# check all goals are found
		amountFound = 0
		for go in goals:
			if go.found:
				amountFound += 1
		if amountFound == 4 and self.allFound == 0:
			# found all the poles so set the destination to be the location of the poles
			self.allFound = 1
			print("Found all poles")
			robotdestination = []
			for goa in goals:
				# add to the list if the robot is not already found
				robotdestination.append(goa.location)
			self.destinations = robotdestination
			print("Changing destinations to pole locations: " + str(
				robotdestination) + "The current destionations are:" + str(self.destinations))
			self.nextDestination()
		# display the depth and rgb camera images
		if dres is not None:
			cv2.imshow("depth", dres)
		cv2.imshow("window", res)
		cv2.waitKey(3)

	def cancelAllGoals(self):
		self.client.cancel_all_goals()
		self.client.wait_for_result()
		self.nextDestination()

	def nextDestination(self):
		print("Current Goal: " + str(self.currentGoal) + " current desintaions :" + str(self.destinations))
		if (len(self.destinations) >= 1):
			self.currentGoal = self.destinations.pop()
			self.Map_convert(self.currentGoal)
		else:
			print("Finished?")

	def depth_callback(self, msg):
		self.DepthimageMesaures = self.bridge.imgmsg_to_cv2(msg, "passthrough")
		self.Depthimage = self.bridge.imgmsg_to_cv2(msg, "32FC1")

	def costmap_callback(self, msg):
		self.resolution = msg.info.resolution
		self.origin = msg.info.origin
		data = np.asarray(msg.data, dtype=np.uint8).reshape(msg.info.height, msg.info.width)
		self.dilatedCostmap = cv2.dilate(data, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)), iterations=3)
		if len(self.destinations) == 0:
			for y in range(35, msg.info.height - 35, 20):
				for x in range(35, msg.info.width - 35, 20):
					if self.dilatedCostmap[y][x] <= 0:
						self.destinations.append((y, x))
						self.dilatedCostmap[y][x] = 255
		cv2.imshow('com', self.dilatedCostmap)
		print(self.destinations)
		self.nextDestination()

	def rotate(self):
		# Starts a new node
		vel_msg = Twist()

		# Receiveing the user's input
		print("Let's rotate your robot")
		speed = 120
		angle = 360
		clockwise = True

		# Converting from angles to radians
		angular_speed = speed * 2 * math.pi / 360
		relative_angle = angle * 2 * math.pi / 360

		# We wont use linear components
		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0

		# Checking if our movement is CW or CCW
		if clockwise:
			vel_msg.angular.z = -abs(angular_speed)
		else:
			vel_msg.angular.z = abs(angular_speed)
		# Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_angle = 0

		while (current_angle < relative_angle):
			self.cmd_vel_pub.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed * (t1 - t0)

		# Forcing our robot to stop
		vel_msg.angular.z = 0
		self.cmd_vel_pub.publish(vel_msg)
		rospy.spin()


rospy.init_node('follower')
follower = self()
rospy.spin()
# END ALL
