#!/usr/bin/env python3

# Test script to identify vision kits orientation and coordinate frames!

import sys
import capnp
import pathlib
import rospy
import tf

import ecal.core.core as ecal_core


from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from vision.msg import VioState
from byte_subscriber import ByteSubscriber

current_path = str(pathlib.Path(__file__).parent.resolve())
capnp_schema_path = current_path + '/../src/capnp'
capnp.add_import_hook([capnp_schema_path])

import odometry3d_capnp as eCALOdometry3d

class RosOdometryPublisher:

    def __init__(self):

        self.ros_odom_pub = rospy.Publisher('/vio/data', Odometry, queue_size=10)
        self.viostate_pub = rospy.Publisher('/vision/vio_state', VioState, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
 
    def callback(self, topic_name, msg, time):
      
        with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:

            ros_msg = Odometry();
            ros_msg.header.seq = odometryMsg.header.seq
            ros_msg.header.stamp = rospy.Time.now()
            ros_msg.header.frame_id = "map"
            ros_msg.child_frame_id = "vio_frame"
            ros_msg.pose.pose.position.x = odometryMsg.pose.position.x
            ros_msg.pose.pose.position.y = odometryMsg.pose.position.y
            ros_msg.pose.pose.position.z = odometryMsg.pose.position.z
            ros_msg.pose.pose.orientation.w = odometryMsg.pose.orientation.w
            ros_msg.pose.pose.orientation.x = odometryMsg.pose.orientation.x
            ros_msg.pose.pose.orientation.y = odometryMsg.pose.orientation.y
            ros_msg.pose.pose.orientation.z = odometryMsg.pose.orientation.z
            # ros_msg.pose.covariance = odometryMsg.poseCovariance

            ros_msg.twist.twist.linear.x = odometryMsg.twist.linear.x
            ros_msg.twist.twist.linear.y = odometryMsg.twist.linear.y
            ros_msg.twist.twist.linear.z = odometryMsg.twist.linear.z
            ros_msg.twist.twist.angular.x = odometryMsg.twist.angular.x
            ros_msg.twist.twist.angular.y = odometryMsg.twist.angular.y
            ros_msg.twist.twist.angular.z = odometryMsg.twist.angular.z
            # ros_msg.twist.covariance = odometryMsg.twistCovariance

            tf_pose = ros_msg.pose.pose
                
            tf = TransformStamped()
            tf.header.stamp = rospy.Time.now()
            tf.header.frame_id = ros_msg.header.frame_id
            tf.child_frame_id = ros_msg.child_frame_id
            tf.transform.translation.x = tf_pose.position.x
            tf.transform.translation.y = tf_pose.position.y
            tf.transform.translation.z = tf_pose.position.z
            tf.transform.rotation.x = tf_pose.orientation.x
            tf.transform.rotation.y = tf_pose.orientation.y
            tf.transform.rotation.z = tf_pose.orientation.z
            tf.transform.rotation.w = tf_pose.orientation.w
                
            self.tf_broadcaster.sendTransform(
                (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z),
                (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w),
                tf.header.stamp,
                tf.child_frame_id,
                tf.header.frame_id
            )

            vio_state = VioState();
            vio_state.header.seq = odometryMsg.header.seq
            vio_state.header.stamp = rospy.Time.now()
            vio_state.header.frame_id = "odom"
            vio_state.vision_failure = odometryMsg.metricVisionFailureLikelihood
            vio_state.inertial_failure = odometryMsg.metricInertialFailureLikelihood
            vio_state.failure_drift = odometryMsg.estimatedFailureModeDrift
            vio_state.vio_failure = odometryMsg.metricFailureVio

            self.viostate_pub.publish(vio_state)
            self.ros_odom_pub.publish(ros_msg)
            rospy.loginfo("eCAL-ROS Bridge Active")

def main():  

    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    ecal_core.initialize(sys.argv, "test_odometry_sub")
    ecal_core.set_process_state(1, 1, "I feel good")
    rospy.init_node("ros_odometry_publisher")

    ecal_topic = "S1/vio_odom"
    print(f"ecal-ros bridge subscribe topic: {ecal_topic}")

    ros_odometry_publisher = RosOdometryPublisher()
    sub = ByteSubscriber(ecal_topic)
    sub.set_callback(ros_odometry_publisher.callback)

    rospy.spin()
    ecal_core.finalize()

if __name__ == "__main__":
    main()