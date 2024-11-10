#!/usr/bin/env python3

import rospy
import csv
from datetime import datetime
from nav_msgs.msg import Odometry
from vision.msg import VioState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

def generate_timestamped_filename():
    timestamp = datetime.now().strftime("%Y%m%d%H%M")
    return f"logger_{timestamp}.csv"

def create_csv_files():
    # Create separate CSV files for each data type
    vicon_file = open('vicon.csv', mode='w', newline='')
    vio_file = open('vio.csv', mode='w', newline='')
    ekf_file = open('ekf.csv', mode='w', newline='')
    vio_state_file = open('vio_state.csv', mode='w', newline='')

    vicon_writer = csv.writer(vicon_file)
    vio_writer = csv.writer(vio_file)
    ekf_writer = csv.writer(ekf_file)
    vio_state_writer = csv.writer(vio_state_file)

    # Write headers for each CSV file
    vicon_writer.writerow(["Time", "Sequence", "Stamp", "Frame_ID", "Position_x", "Position_y", "Position_z", 
                           "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w"])
    vio_writer.writerow(["Time", "Sequence", "Stamp", "Frame_ID", "Child_Frame_ID", "Position_x", "Position_y", 
                         "Position_z", "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w", 
                         "L.Velocity_x", "L.Velocity_y", "L.Velocity_z", "A.Velocity_x", "A.Velocity_y", 
                         "A.Velocity_z"])
    ekf_writer.writerow(["Time", "Sequence", "Stamp", "Frame_ID", "Position_x", "Position_y", "Position_z", 
                         "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w"])
    vio_state_writer.writerow(["Time", "Sequence", "Timestamp", "Frame_ID", "Vision Failure", "Inertial Failure", 
                               "Failure Drift", "VIO Failure", "Reset Counter"])

    return vicon_writer, vio_writer, ekf_writer, vio_state_writer

def vicon_callback(msg, vicon_writer):
    vicon_writer.writerow([rospy.get_time(), msg.header.seq, msg.header.stamp, msg.header.frame_id, msg.pose.position.x,
                           msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y,
                           msg.pose.orientation.z, msg.orientation.w])

def odometry_callback(msg, vio_writer):
    vio_writer.writerow([rospy.get_time(), msg.header.seq, msg.header.stamp, msg.header.frame_id, msg.child_frame_id,
                         msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                         msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w, msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                         msg.twist.twist.linear.z, msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                         msg.twist.twist.angular.z])

def ekf_callback(msg, ekf_writer):
    ekf_writer.writerow([rospy.get_time(), msg.header.seq, msg.header.stamp, msg.header.frame_id, msg.pose.position.x,
                         msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y,
                         msg.pose.orientation.z, msg.orientation.w])

def viostate_callback(msg, vio_state_writer):
    vio_state_writer.writerow([rospy.get_time(), msg.header.seq, msg.header.stamp, msg.header.frame_id, 
                               msg.vision_failure, msg.inertial_failure, msg.failure_drift, msg.vio_failure, 
                               msg.reset_counter])

def listener():
    rospy.init_node('logger', anonymous=False)

    # Create CSV files and writers
    vicon_writer, vio_writer, ekf_writer, vio_state_writer = create_csv_files()

    # Subscribe to the relevant topics
    rospy.Subscriber('/vision/vio_state', VioState, lambda msg: viostate_callback(msg, vio_state_writer))
    # rospy.Subscriber('/vrpn_client_node/Akira/pose', PoseStamped, lambda msg: vicon_callback(msg, vicon_writer))
    # rospy.Subscriber('S0/basalt/odom', Odometry, lambda msg: odometry_callback(msg, vio_writer))
    # rospy.Subscriber('/mavros/local_position/pose', PoseStamped, lambda msg: ekf_callback(msg, ekf_writer))

    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Starting the logger node...")
    listener()
    rospy.loginfo("Logging in process...")
