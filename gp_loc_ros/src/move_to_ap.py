#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math

class NavigatetoAPRobot:
    def __init__(self):
        rospy.init_node('move_to_ap_robot')
        self.robot_name = rospy.get_param('~robot_name', 'tb2_2')

        # Create action client for move base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Subscriber for odom
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
        # Subscriber for ap_pos
        self.ap_pos_subscriber = rospy.Subscriber('ap_pos', Point, self.ap_pos_callback)

        # Latest messages from subscriptions
        self.latest_odom = None
        self.latest_ap_pos = None

        # Time delta for goal point calculation
        self.delta_t = 3

    def odom_callback(self, msg):
        self.latest_odom = msg.pose.pose.position

    def ap_pos_callback(self, msg):
        self.latest_ap_pos = msg
        # print(self.latest_odom,self.latest_ap_pos)
        if self.latest_odom is not None:
            goal_point = self.calc_goal_point(self.latest_ap_pos, self.latest_odom)
            print("GOAL:",goal_point)
            self.send_goal(goal_point)

    def calc_goal_point(self, ap_pos, odom_pos):
        x_vel = ap_pos.x - odom_pos.x
        y_vel = ap_pos.y - odom_pos.y

        mag = math.sqrt(x_vel**2 + y_vel**2 )

        goal_point = Point()
        goal_point.x = odom_pos.x + (x_vel/mag)*self.delta_t
        goal_point.y = odom_pos.y +(y_vel/mag)*self.delta_t

        return goal_point

    def send_goal(self, goal_point):
        if goal_point is not None:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.robot_name+'/map'  # Assuming the goal_point is provided in 'map' frame
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = goal_point
            goal.target_pose.pose.position.x = goal_point.x
            goal.target_pose.pose.position.y = goal_point.y
            goal.target_pose.pose.orientation.w = 1.0

            self.client.send_goal(goal)
            wait = self.client.wait_for_result(rospy.Duration.from_sec(2.0))

            # if not wait:
            #     rospy.logerr("Action server not available!")
            #     rospy.signal_shutdown("Action server not available!")
            # else:
            #     return self.client.get_result()

if __name__ == '__main__':
    try:
        navigator = NavigatetoAPRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
