#!/usr/bin/env python3
#_________________________________________________#
#
# Current script - Stable
# Subscribe odometry from /S0/basalt/odom to ensure proper data retrieval
# In addition to publish data to /mavros/odometry/out, data is also 
# broadcasted to tf for frames base_link and odom to complete the tf tree
#
#_________________________________________________#

import rospy
import tf
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import CompanionProcessStatus

class TFToOdometry:
    def __init__(self):

        rospy.init_node('vio_mavros_bridge',anonymous = False)      
        self.odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)
        self.cps_pub = rospy.Publisher('/mavros/companion_process/status', CompanionProcessStatus, queue_size=10)        
        self.latest_odom_data = None
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/S0/basalt/odom', Odometry, self.odometry_callback)
        self.rate = rospy.Rate(40)    #rate control for publish rate
        
    def odometry_callback(self, odom_data):

        self.latest_odom_data = odom_data
        
    def run(self):
    
        while not rospy.is_shutdown():
            if self.latest_odom_data:
    
                odometry_msg = Odometry()
                odom_data = self.latest_odom_data
                
                odometry_msg.header = odom_data.header
                odometry_msg.header.stamp = rospy.Time.now()
                odometry_msg.header.frame_id = "odom"
                odometry_msg.child_frame_id = "base_link" 
                odometry_msg.pose.pose.position.x = odom_data.pose.pose.position.x
                odometry_msg.pose.pose.position.y = (odom_data.pose.pose.position.y)
                odometry_msg.pose.pose.position.z = odom_data.pose.pose.position.z
                odometry_msg.pose.pose.orientation.x = odom_data.pose.pose.orientation.x
                odometry_msg.pose.pose.orientation.y = odom_data.pose.pose.orientation.y
                odometry_msg.pose.pose.orientation.z = odom_data.pose.pose.orientation.z
                odometry_msg.pose.pose.orientation.w = odom_data.pose.pose.orientation.w
                
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
                  
                cps_msg = CompanionProcessStatus()
                cps_msg.header.stamp = rospy.Time.now()
                cps_msg.state = CompanionProcessStatus.MAV_STATE_ACTIVE
                cps_msg.component = CompanionProcessStatus.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
                
                self.cps_pub.publish(cps_msg)
                self.odom_pub.publish(odometry_msg)
                
                rospy.loginfo("VIO-MAVROS Bridge Active")
            self.rate.sleep()

if __name__ == '__main__':
    node = TFToOdometry()
    node.run()

