#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from gp_model_prediction import GP_Model

def APPredictionNode():
    # Initialize the ROS node with a name
    rospy.init_node('ap_prediction_node', anonymous=True)
    robot_name = rospy.get_param('~robot_name', 'tb2_2')
    robot_number = int(rospy.get_param('~robot_number', '1'))
    robot_gp_model = GP_Model()
    robot_gp_model.load_model(robot_name+'_all_positive_matern')

    # Create a publisher
    pub = rospy.Publisher('ap_pos', PointStamped, queue_size=10)

    # Set the rate of publishing messages
    rate = rospy.Rate(10)  # 10hz
    ap_pos = robot_gp_model.get_ap_pos(robot_number)


    while not rospy.is_shutdown():
        # Create a message. For a Point, we need to specify x, y, and z
        ap_pos_msg = PointStamped()
        ap_pos_msg.point.x = ap_pos[0]
        ap_pos_msg.point.y = ap_pos[1]
        if robot_number==0:
            ap_pos_msg.point.x+=0.5
            ap_pos_msg.point.y+=1
        ap_pos_msg.header.stamp = rospy.Time.now()
        ap_pos_msg.header.frame_id = robot_name+'/map'

        # Publish the message
        pub.publish(ap_pos_msg)

        # Log the message to console
        rospy.loginfo("AP_Position: %s", ap_pos_msg)

        # Maintain the rate of publishing
        rate.sleep()

if __name__ == '__main__':
    try:
        APPredictionNode()
    except rospy.ROSInterruptException:
        pass
