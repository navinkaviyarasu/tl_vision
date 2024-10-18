#!/usr/bin/env python
#_________________________________________________#
#
# Current script - Beta
#
#_________________________________________________#
import rospy
import tf
#from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
#from mavros_msgs.msg import CompanionProcessStatus

class ViconOdometry:
    def __init__(self):

        rospy.init_node('vicon_mavros_bridge',anonymous = False)      
        self.odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)
        #self.cps_pub = rospy.Publisher('/mavros/companion_process/status', CompanionProcessStatus, queue_size=10)
        
        self.latest_transform = None
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/vrpn_client_node/Akira/pose', PoseStamped, self.vicon_callback)
        #self.rate = rospy.Rate(40)
        
    def vicon_callback(self, vicon_data):

        self.latest_transform = vicon_data


    def run(self):

        while not rospy.is_shutdown():
            if self.latest_transform:
    
                odometry_msg = Odometry()
                transform = self.latest_transform
                
                odometry_msg.header = transform.header
                odometry_msg.header.stamp = rospy.Time.now()
                odometry_msg.header.frame_id = "odom"
                odometry_msg.child_frame_id = "base_link"     
                odometry_msg.pose.pose.position.x = transform.pose.position.x
                odometry_msg.pose.pose.position.y = transform.pose.position.y
                odometry_msg.pose.pose.position.z = transform.pose.position.z
                odometry_msg.pose.pose.orientation.x = transform.pose.orientation.x
                odometry_msg.pose.pose.orientation.y = transform.pose.orientation.y
                odometry_msg.pose.pose.orientation.z = transform.pose.orientation.z
                odometry_msg.pose.pose.orientation.w = transform.pose.orientation.w
                
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

                #cps_msg = CompanionProcessStatus()
                #cps_msg.header.stamp = rospy.Time.now()
                #cps_msg.state = CompanionProcessStatus.MAV_STATE_ACTIVE
                #cps_msg.component = CompanionProcessStatus.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
                
                #self.cps_pub.publish(cps_msg)
                self.odom_pub.publish(odometry_msg)
                
                rospy.loginfo("VICON-MAVROS Bridge Active")
            #self.rate.sleep()

if __name__ == '__main__':
    node = ViconOdometry()
    node.run()

