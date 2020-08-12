import time
import numpy as np

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from tracker.filters.robot_kalman_filter import RobotFilter
from tracker.vision.vision_receiver import VisionReceiver
from tracker.constants import TrackerConst


class ROSTracker:
    MAX_ROBOT_PER_TEAM = TrackerConst.MAX_ROBOT_PER_TEAM

    STATE_PREDICTION_TIME = TrackerConst.STATE_PREDICTION_TIME

    MAX_UNDETECTED_DELAY = TrackerConst.MAX_UNDETECTED_DELAY

    def __init__(self, name, vision_address):
        rospy.init_node(name)
        rospy.loginfo('Created ros node {}'.format(name))

        # init dict to store all publishers
        self.pub_dict = {}

        self.last_sending_time = time.time()

        self.vision_receiver = VisionReceiver(vision_address)
        rospy.loginfo('VisionReceiver created. ({}:{})'.format(*vision_address))

        self.robots = [RobotFilter() for _ in range(ROSTracker.MAX_ROBOT_PER_TEAM)]

        self._current_timestamp = None

    @property
    def current_timestamp(self):
        return self._current_timestamp

    def start(self):
        self.vision_receiver.start()
        self.tracker_main_loop()

    def tracker_main_loop(self):
        while not rospy.is_shutdown():

            detection_frame = self.vision_receiver.get()
            self._current_timestamp = detection_frame.t_capture

            for robot_obs in detection_frame.robots_blue:
                obs_state = np.array([robot_obs.x, robot_obs.y, robot_obs.orientation])
                self.robots[robot_obs.robot_id].update(obs_state, detection_frame.t_capture)
                self.robots[robot_obs.robot_id].predict(ROSTracker.STATE_PREDICTION_TIME)

            for robot_obs in detection_frame.robots_yellow:
                obs_state = np.array([robot_obs.x, robot_obs.y, robot_obs.orientation])
                self.robots[robot_obs.robot_id].update(obs_state, detection_frame.t_capture)
                self.robots[robot_obs.robot_id].predict(ROSTracker.STATE_PREDICTION_TIME)

            self.remove_undetected_robot()

            if self.robots:
                self.pub_odom_msgs(self.robots)
            else:
                rospy.logwarn("No robots found...")

    def remove_undetected_robot(self):
        for robot in self.robots:
            if robot.last_t_capture + ROSTracker.MAX_UNDETECTED_DELAY < self.current_timestamp:
                robot.is_active = False

    def pub_odom_msgs(self, robots):
        for robot_id, robot in enumerate(robots):
            if robot.is_active:
                if robot_id not in self.pub_dict:
                    topic_name = "tracked_robot_" + str(robot_id)
                    self.pub_dict[robot_id] = rospy.Publisher(topic_name, Odometry, queue_size=10)
                    rospy.loginfo('Created new topic {}'.format(topic_name))

                # get position and kinematic info from robot
                robot_pose = tuple(robot.pose)  # position (x, y, yaw)
                robot_vel = tuple(robot.velocity)  # velocity (x, y, yaw)

                # create odom message
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()  # off by ~2/100 sec compared to self.current_timestamp

                # convert yaw to euler to quaternion
                quat = tf.transformations.quaternion_from_euler(0, 0, robot.get_orientation)

                # set the pose
                odom.pose.pose = Pose(Point(robot_pose[0] / 1000, robot_pose[1] / 1000, 0), Quaternion(*quat))

                # set the twist (first part is linear velocity, second is angular)
                odom.twist.twist = Twist(Vector3(robot_vel[0] / 1000, robot_vel[1] / 1000, 0), Vector3(0, 0, robot_vel[2]))

                # publish message
                self.pub_dict[robot_id].publish(odom)
