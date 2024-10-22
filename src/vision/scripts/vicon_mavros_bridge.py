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
from geometry_msgs.msg import TransformStamped

class ViconOdometry:
    def __init__(self):

        rospy.init_node('vicon_mavros_bridge',anonymous = False)      
        self.odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)        
        self.latest_vicon_data = None
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/vrpn_client_node/Akira/pose', PoseStamped, self.vicon_callback)
        #self.rate = rospy.Rate(40)
        
    def vicon_callback(self, vicon_data):

        self.latest_vicon_data = vicon_data


    def run(self):

        while not rospy.is_shutdown():
            if self.latest_vicon_data:
    
                odometry_msg = Odometry()
                vicon_data = self.latest_vicon_data
                
                odometry_msg.header = vicon_data.header
                odometry_msg.header.stamp = rospy.Time.now()
                odometry_msg.header.frame_id = "odom"
                odometry_msg.child_frame_id = "base_link"     
                odometry_msg.pose.pose.position.x = vicon_data.pose.position.x
                odometry_msg.pose.pose.position.y = vicon_data.pose.position.y
                odometry_msg.pose.pose.position.z = vicon_data.pose.position.z
                odometry_msg.pose.pose.orientation.x = vicon_data.pose.orientation.x
                odometry_msg.pose.pose.orientation.y = vicon_data.pose.orientation.y
                odometry_msg.pose.pose.orientation.z = vicon_data.pose.orientation.z
                odometry_msg.pose.pose.orientation.w = vicon_data.pose.orientation.w
                
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

