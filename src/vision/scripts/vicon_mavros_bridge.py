#!/usr/bin/env python
#_________________________________________________#
#
# Current script - Stable
#
#_________________________________________________#
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped

class ViconOdometry:
    def __init__(self):

        rospy.init_node('vicon_mavros_bridge',anonymous = False)      
        self.odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)        
        self.latest_pose_data = None
        self.latest_twist_data = None
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/vrpn_client_node/Akira/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/vrpn_client_node/Akira/twist', TwistStamped,self.twist_callback)
        #self.rate = rospy.Rate(40)
        
    def pose_callback(self, pose_data):
        self.latest_pose_data = pose_data

    def twist_callback(self, twist_data):
        self.latest_twist_data = twist_data


    def run(self):

        while not rospy.is_shutdown():
            if self.latest_pose_data and self.latest_twist_data:
    
                odometry_msg = Odometry()
                pose_data = self.latest_pose_data
                twist_data = self.latest_twist_data
                
                odometry_msg.header = pose_data.header
                odometry_msg.header.stamp = rospy.Time.now()
                odometry_msg.header.frame_id = "odom"
                odometry_msg.child_frame_id = "base_link"     
                odometry_msg.pose.pose.position.x = pose_data.pose.position.x
                odometry_msg.pose.pose.position.y = pose_data.pose.position.y
                odometry_msg.pose.pose.position.z = pose_data.pose.position.z
                odometry_msg.pose.pose.orientation.x = pose_data.pose.orientation.x
                odometry_msg.pose.pose.orientation.y = pose_data.pose.orientation.y
                odometry_msg.pose.pose.orientation.z = pose_data.pose.orientation.z
                odometry_msg.pose.pose.orientation.w = pose_data.pose.orientation.w
                odometry_msg.twist.twist.linear.x = twist_data.twist.linear.x
                odometry_msg.twist.twist.linear.y = twist_data.twist.linear.y
                odometry_msg.twist.twist.linear.z = twist_data.twist.linear.z
                odometry_msg.twist.twist.angular.x = twist_data.twist.angular.x
                odometry_msg.twist.twist.angular.y = twist_data.twist.angular.y
                odometry_msg.twist.twist.angular.z = twist_data.twist.angular.z
                
                tf_pose = odometry_msg.pose.pose
                
                tf = TransformStamped()
                tf.header.stamp = rospy.Time.now()
                tf.header.frame_id = odometry_msg.header.frame_id
                tf.child_frame_id = odometry_msg.child_frame_id
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

                self.odom_pub.publish(odometry_msg)
                
                rospy.loginfo("VICON-MAVROS Bridge Active")
            #self.rate.sleep()

if __name__ == '__main__':
    node = ViconOdometry()
    node.run()

