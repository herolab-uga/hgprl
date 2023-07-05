#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
import numpy as np

class PosPrediction:
    def __init__(self):
        rospy.init_node('pos_prediction', anonymous=True)
        # Initialize global storage for positions
        self.robot_number = int(rospy.get_param('~robot_number', '1'))
        self.robot_count = int(rospy.get_param('~robot_count', '3'))

        self.AP_positions = np.zeros((2, self.robot_count))  # We're assuming 2D positions
        self.local_positions = np.zeros((2, self.robot_count))


        # Create subscribers for each robot
        for i in range(self.robot_count):
            rospy.Subscriber("/tb2_"+str(i)+"/ap_pos", PointStamped, self.callback, (i,))
        # rospy.Subscriber("/tb2_1/ap_pos", Point, self.callback, (1,))
        # rospy.Subscriber("/tb2_2/ap_pos", Point, self.callback, (2,))

        # Create subscribers for each robot's odometry
        for i in range(self.robot_count):
            rospy.Subscriber("/tb2_"+str(i)+"/odom", Odometry, self.odom_callback, (i,))
        # rospy.Subscriber("/tb2_1/odom", Odometry, self.odom_callback, (1,))
        # rospy.Subscriber("/tb2_2/odom", Odometry, self.odom_callback, (2,))


        # Create publishers for each robot's local position
        self.pub = []
        for i in range(self.robot_count):
            # if i !=self.robot_number:
            self.pub.append(rospy.Publisher('tb2_'+str(i)+'_rel_pos', PointStamped, queue_size=10))
        # self.pub.append(rospy.Publisher("/tb2_1_rel_pos", Point, queue_size=10))
        # self.pub.append(rospy.Publisher("/tb2_2_rel_pos", Point, queue_size=10))
    
    def odom_callback(self, data, args):
        # Extract robot index from callback args
        robot_index = args[0]

        # Store the received data into global local_positions
        self.local_positions[0][robot_index] = data.pose.pose.position.x
        self.local_positions[1][robot_index] = data.pose.pose.position.y
        self.calculate_relative_positions()

        # self.publish_relative_positions()

    def callback(self, data, args):
        # Extract robot index from callback args
        robot_index = args[0]

        # Store the received data into global AP_positions
        self.AP_positions[0][robot_index] = data.point.x
        self.AP_positions[1][robot_index] = data.point.y
        self.calculate_relative_positions()
    def calculate_relative_positions(self):
        self.pred_local_positions = np.zeros((2, self.robot_count))
        # Perform transformations and publish local positions
        for j in range(self.robot_count):
            if j != self.robot_number:
                ap_diff = [0, 0]
                ap_diff[0] = self.AP_positions[0][self.robot_number] - self.AP_positions[0][j]
                ap_diff[1] = self.AP_positions[1][self.robot_number] - self.AP_positions[1][j]
                self.pred_local_positions[0][j] = self.local_positions[0][j] + ap_diff[0]
                self.pred_local_positions[1][j] = self.local_positions[1][j] + ap_diff[1]

                # Publish local positions
                local_pos_msg = PointStamped()
                local_pos_msg.point.x = self.pred_local_positions[0][j]
                local_pos_msg.point.y = self.pred_local_positions[1][j]
                local_pos_msg.header.stamp = rospy.Time.now()
                local_pos_msg.header.frame_id = "tb2_"+str(self.robot_number)+'/map'
                self.pub[j].publish(local_pos_msg)

if __name__ == '__main__':
    transformer = PosPrediction()
    rospy.spin()