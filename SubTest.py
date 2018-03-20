#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import Extras as extra
import numpy as np
import actionlib
import math
from math import sin, cos, radians, pi
import tf2_ros
import tf


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


class Follower:
    np.set_printoptions(threshold=numpy.nan)
    current_x = 0
    current_y = 0
    current_rad = 0
    resolution = 0.05
    origin = 0
    Map = 0
    CostMap = 0
    Header = 0
    width = 216.0
    Depthimage = None
    DepthimageMesaures = 0
    height = 250.0
    distance = 0
    numofPointVisited = 0
    cordsList = []
    bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    twist = Twist()
    Blue = Goal("Blue", 110, 120)
    Yellow = Goal("Yellow", 25, 35)
    Green = Goal("Green", 60, 70)
    Red = Goal("Red", 0, 2)
    foundColour = 0
    running = 0
    center = 0
    MaxDest = 0
    CurrentDest = 0
    destinations = []
    currentGoal = None
    fixedPoints = [(1.09, -4.18), (1.011, -2.00), (-4.42, 0.20), (-4.32, 4.93), (1.62, 4.68)]
    fixed = 1

    def __init__(self):
        rospy.set_param('/turtlebot/move_base/TrajectoryPlannerROS/occdist_scale', 1.0)
        rospy.set_param('/turtlebot/move_base/TrajectoryPlannerROS/pdist_scale', 5.00)
        #rospy.set_param('/turtlebot/move_base/TrajectoryPlannerROS/gdist_scale', 5.00)
        rospy.set_param('/turtlebot/move_base/TrajectoryPlannerROS/min_vel_x', 1.00)
        rospy.set_param('/turtlebot/move_base/TrajectoryPlannerROS/max_vel_x', 5.00)
        self.client = actionlib.SimpleActionClient('turtlebot/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=1)

        self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/turtlebot/camera/depth/image_raw', Image, self.depth_callback)

        # self.map = rospy.Subscriber('/map', OccupancyGrid, self.Map_callback)
        self.costMap = rospy.Subscriber('/turtlebot/move_base/global_costmap/costmap', OccupancyGrid,
                                        self.costmap_callback)
        self.twist = Twist()
        self.goal = MoveBaseGoal()

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            try:
                test = tfBuffer.lookup_transform('map', 'turtlebot/base_footprint', rospy.Time())
                # tf.transformations.
                pos_x = test.transform.translation.x
                pos_y = test.transform.translation.y
                q_w = test.transform.rotation.w
                q_x = test.transform.rotation.x
                q_y = test.transform.rotation.y
                q_z = test.transform.rotation.z
                # print(test)
                roll, putch, yaw = extra.quaternion_to_euler_angle(q_w, q_x, q_y, q_z)
                # print (pos_x,pos_y,roll,putch,yaw)
                Follower.current_x = pos_x
                Follower.current_y = pos_y
                Follower.current_rad = yaw

                goals = [self.Blue, self.Yellow, self.Red, self.Green]
                for goal in goals:
                    if goal.location is not None:
                        #have current location
                        (x,y) = goal.location
                        if not goal.shownFound:
                            if(pos_x+1 > x and x > pos_x-1):
                                if(pos_y+1 > y and y > pos_y-1):
                                    goal.shownFound = True
                                    print("With in" + goal.name)
                                    self.rotate()


                rate.sleep()

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

    def Map_callback(self, msg):
        MapInfo = msg.info
        Follower.Header = msg.header
        MapData = msg.data
        Follower.resolution = msg.info.resolution
        Follower.origin = msg.info.origin
        size = len(MapData)
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height,
                                                           msg.info.width)  # convert map to numpy array
        Follower.Map = data
        for i in range(20, msg.info.height, 30):
            for j in range(20, msg.info.width, 30):
                if (self.CostMapcheck(i, j)):
                    Follower.cordsList.append((i, j))
        # self.Map_convert(Follower.cordsList[0][0],Follower.cordsList[0][1])
        MaxDest = len(self.cordsList)


    def Map_convert(self, (goal_x, goal_y)):
        if self.fixed == 0:
            x = Follower.origin.position.x + (goal_x * Follower.resolution);
            y = Follower.origin.position.y + (goal_y * Follower.resolution);
        else:
            x = goal_x
            y = goal_y
        print("GoalX:" + str(goal_x) + " X: " + str(x) + "GoalY: " + str(goal_y) + " Y:" + str(y))
        self.moveToGoal(x, y)

    def moveToGoal(self, xGoal, yGoal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = xGoal
        goal.target_pose.pose.position.y = yGoal
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal, done_cb=self.DoneCB, feedback_cb=self.FeedbackCB)

    def DoneCB(self, state, result):
        print("Done callback" + str(result))
        if (state != 2):
            # not cancelled
            if len(self.destinations) >= 1:
                #self.spinInCircle()
                self.currentGoal = self.destinations.pop(0)
                self.Map_convert(self.currentGoal)
            else:
                #goals are finished
                pb=0



    def ActiveCB(self):
        print("Act callback")

    def FeedbackCB(self, msg):
        daksdj = "dsd"


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        depth = Follower.Depthimage

        self.Blue.UpdateMask(hsv)
        self.Green.UpdateMask(hsv)
        self.Yellow.UpdateMask(hsv)
        self.Red.UpdateMask(hsv)

        addedMask = self.Blue.mask + self.Yellow.mask + self.Red.mask + self.Green.mask

        res = cv2.bitwise_and(image, image, mask=addedMask)

        if Follower.Depthimage is not None:
            dres = cv2.bitwise_and(Follower.Depthimage, Follower.Depthimage, mask=addedMask)
            # print(Follower.DepthimageMesaures.shape)
            depth = cv2.bitwise_and(Follower.Depthimage, Follower.DepthimageMesaures, mask=addedMask)
            depth[depth == 0] = np.nan
            average_depth = np.nanmean(depth)
            if average_depth <= 1:
                print "Found an object"
                self.rotate()
            y = 0.0
            if not np.isnan(average_depth):
                x, y = self.point_pos(Follower.current_x, Follower.current_y, average_depth, Follower.current_rad)
            # print ("I think x:" + str(x) + " and y:" + str(y))
            cv2.imshow("depth", dres)
        goals = [self.Blue, self.Yellow, self.Red, self.Green]
        h, w, d = image.shape
        for goal in goals:
            M = cv2.moments(goal.mask)
            if M['m00'] > 50000.0:  # can see colour

                if Follower.foundColour == 0:
                    Follower.foundColour = 1
                    goal.running = 1
                if (goal.running == 1 and Follower.foundColour == 1 and goal.found == 0):
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
                    # BEGIN CONTROL
                    err = cx - w / 2
                    #self.twist.linear.x = 0.5
                    # self.client.cancel_all_goals()
                    self.twist.angular.z = -float(err) / 200
                    # if (Follower.distance < 2.5):
                    #	self.twist.linear.x = 0
                    self.cmd_vel_pub.publish(self.twist)
                    # print(err)
                    if (err == 0):
                        if (y != 0.0):
                            print(M['m00'])
                            print("found " + goal.name)
                            goal.location = (x, y)
                            self.destinations.append(goal.location)
                            print(goal.location)
                            goal.found = 1
                            Follower.foundColour = 0
                            rospy.sleep(1)

                # centered
            else:
                if (goal.found != 0 and Follower.foundColour == 1):
                    goal.running = 0
                    Follower.foundColour = 0
        # END CONTROL
        cv2.imshow("window", res)
        cv2.waitKey(3)

    def point_pos(self, x, y, d, rad):
        newx = x + d * cos(rad)
        newy = y + d * sin(rad)
        return newx, newy

    def depth_callback(self, msg):
        Follower.DepthimageMesaures = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        Follower.Depthimage = self.bridge.imgmsg_to_cv2(msg, "32FC1")



    def costmap_callback(self, msg):
        Follower.Header = msg.header
        MapData = msg.data
        Follower.resolution = msg.info.resolution
        Follower.origin = msg.info.origin
        data = np.asarray(msg.data, dtype=np.uint8).reshape(msg.info.height, msg.info.width)
        self.costmap = data
        newimg = cv2.dilate(self.costmap, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=5)
        if len(self.destinations) == 0:
            for x in range(35, msg.info.width-35, 20):
                for y in range(35, msg.info.height-35, 20):
                    if newimg[y][x] <= 0:
                        self.destinations.append((y, x))
                        newimg[y][x] = 255
        # print self.destinations
        cv2.imshow('com', newimg)
        self.destinations = self.fixedPoints
        print(self.destinations)
        self.currentGoal = self.destinations.pop(0)
        self.Map_convert(self.currentGoal)

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
follower = Follower()
rospy.spin()
# END ALL
