#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
import numpy as np
import concurrent.futures

class Rendezvous:
    def __init__(self):
        rospy.init_node('rendezvous')
        self.robot_name = rospy.get_param('~robot_name', 'tb2_2')

        # Create action client for move base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Subscriber for odom
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback,queue_size=30)
        # Subscriber for ap_pos
        # self.ap_pos_subscriber = rospy.Subscriber('ap_pos', Point, self.ap_pos_callback)

        self.robot_number = int(rospy.get_param('~robot_number', '1'))
        self.robot_count = int(rospy.get_param('~robot_count', '3'))

        self.local_positions = np.zeros((2, self.robot_count))


        # Create subscribers for each robot
        for i in range(self.robot_count):
            rospy.Subscriber('tb2_'+str(i)+'_rel_pos', PointStamped, self.pos_callback, (i,),queue_size=30)

        # Latest messages from subscriptions
        self.latest_odom = None

        # Time delta for goal point calculation
        self.delta_t = 1
    
    def pos_callback(self, data, args):
        # Extract robot index from callback args
        robot_index = args[0]

        # Store the received data into global local_positions
        self.local_positions[0][robot_index] = data.point.x
        self.local_positions[1][robot_index] = data.point.y
        # print(robot_index, self.local_positions[0][robot_index],self.local_positions[1][robot_index])

    def odom_callback(self, msg):
        self.latest_odom = msg.pose.pose.position
        x_vel,y_vel=self.find_vel(self.latest_odom)
        print(self.latest_odom.x,self.latest_odom.y,x_vel,y_vel)
        if abs(x_vel)<1.5 and abs(y_vel)<1.5:
            self.client.cancel_all_goals()
            print("Rendezvous Happened")
        else:
            goal_point = self.calc_goal_point(self.latest_odom)
            # if self.execution_done:
            #     self.execution_done = False
                # print("GOAL:",goal_point)
                # with concurrent.futures.ThreadPoolExecutor() as executor:
                #     future = executor.submit(self.send_goal, goal_point)
                #     self.execution_done = future.result()
                #     print("Execution Done",self.execution_done)
            self.send_goal(goal_point)

    def find_vel(self,odom_pos):
        # odom_pos = self.latest_odom
        x_vel,y_vel=0,0
        for j in range(self.robot_count):
            if j != self.robot_number:
                x_vel += (self.local_positions[0][j] - odom_pos.x)
                y_vel += (self.local_positions[1][j] - odom_pos.y)
        return x_vel,y_vel

    def calc_goal_point(self,odom_pos):
        # odom_pos = self.latest_odom
        x_vel,y_vel=self.find_vel(odom_pos)
        

        # mag = math.sqrt(x_vel**2 + y_vel**2 )

        goal_point = Point()
        goal_point.x = odom_pos.x + (x_vel/(self.robot_count-1))*self.delta_t
        goal_point.y = odom_pos.y +(y_vel/(self.robot_count-1))*self.delta_t

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
            # self.client.wait_for_result(rospy.Duration.from_sec(2.0))
            # return True

            # if not wait:
            #     rospy.logerr("Action server not available!")
            #     rospy.signal_shutdown("Action server not available!")
            # else:
            #     return self.client.get_result()

if __name__ == '__main__':
    try:
        navigator = Rendezvous()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
