#!/usr/bin/env python3

import rospy
import csv
from std_msgs.msg import String  # Change to the appropriate message type
from sensor_msgs.msg import Imu  # If you're using Imu messages or other types
from nav_msgs.msg import Odometry
from vision.msg import VioState

# Define a callback function that is called when a new message is received
def callback(msg):
    # Open the CSV file in append mode
    with open('ros_messages.csv', mode='a') as file:
        writer = csv.writer(file)
        
        # Check the type of message and extract the fields accordingly
        # if isinstance(msg, VioState):
            # For example, if you're subscribing to a String message
        writer.writerow([rospy.get_time(), msg.header.seq, msg.header.stamp, msg.header.frame_id, msg.vision_failure, msg.inertial_failure, msg.failure_drift, msg.vio_failure, msg.reset_counter])  # Write the timestamp and message content
        rospy.loginfo("Logging in progress...")
    #     elif isinstance(msg, Imu):
    #         # For example, if you're subscribing to an Imu message
    #         writer.writerow([rospy.get_time(), msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
    #                          msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
    #                          msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    #     # Add more conditions here for other message types if needed

    # print(f"Message saved: {msg}")

# Initialize the ROS node
def listener():
    rospy.init_node('ros_to_csv_listener', anonymous=True)

    # Subscribe to the topic of interest (change the topic name and message type)
    rospy.Subscriber("/vision/vio_state", VioState, callback)  # Change String to the correct message type

    # Keep the script running to listen for messages
    rospy.spin()

if __name__ == '__main__':
    listener()
