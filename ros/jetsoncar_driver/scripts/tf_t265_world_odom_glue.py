#!/usr/bin/env python  
import sys
import roslib
import rospy
import math
from tf import transformations as tf
from tf import TransformBroadcaster, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg

broadcaster = None
listener = None

world_frame = ""
t265_world_frame = ""
odom_frame = ""
t265_odom_frame = ""
odom_topic = ""

class OdomGlue:
    def __init__(self, world_frame, t265_world_frame,
                 odom_frame, t265_odom_frame, odom_topic):
        self.world_frame = world_frame
        self.t265_world_frame = t265_world_frame
        self.odom_frame = odom_frame
        self.t265_odom_frame = t265_odom_frame        

        self.broadcaster = TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)        
        #self.listener.waitForTransform(world_frame, t265_world_frame, rospy.Time(0), rospy.Duration(100))
        #self.listener.waitForTransform(odom_frame, t265_odom_frame, rospy.Time(0), rospy.Duration(100))

        rospy.Subscriber(odom_topic, nav_msgs.msg.Odometry, self.odometry_callback)
        #self.publish_timer = rospy.Timer(rospy.Duration(0.01), self.odometry_callback)

    def odometry_callback(self, msg):
        world_T_t265 = None
        odom_T_t265 = None

        #self.listener.waitForTransform(world_frame, t265_world_frame, rospy.Time(0), rospy.Duration(1))
        #self.listener.waitForTransform(odom_frame, t265_odom_frame, rospy.Time(0), rospy.Duration(1))

        # See the tf.transformations library documentation: http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
        try:
            #(trans, rot) = self.listener.lookupTransform(self.world_frame, self.t265_world_frame, rospy.Time(0))
            T = self.tfBuffer.lookup_transform(self.world_frame, self.t265_world_frame, rospy.Time())

            trans = (T.transform.translation.x, T.transform.translation.y, T.transform.translation.z)
            rot = (T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z, T.transform.rotation.w)

            world_T_t265 = tf.concatenate_matrices(tf.translation_matrix(trans), tf.quaternion_matrix(rot))
            #t265_T_world = tf.inverse_matrix(world_T_t265)
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.logwarn("OdomGlue: Could not lookup world to T265 transform")    
            return

        try:
            #(trans, rot) = self.listener.lookupTransform(self.odom_frame, self.t265_odom_frame, rospy.Time(0))
            T = self.tfBuffer.lookup_transform(self.odom_frame, self.t265_odom_frame, rospy.Time())

            trans = (T.transform.translation.x, T.transform.translation.y, T.transform.translation.z)
            rot = (T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z, T.transform.rotation.w)

            odom_T_t265 = tf.concatenate_matrices(tf.translation_matrix(trans), tf.quaternion_matrix(rot))
            t265_T_odom = tf.inverse_matrix(odom_T_t265)
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.logwarn("OdomGlue: Could not lookup odom to T265 transform")    
            return

        world_T_odom = tf.concatenate_matrices(world_T_t265, t265_T_odom)

        translation = (0.0, 0.0, 0.0)
        rotation = (0.0, 0.0, 0.0, 1.0)

        try:
            self.broadcaster.sendTransform(tf.translation_from_matrix(world_T_odom), tf.quaternion_from_matrix(world_T_odom), rospy.Time.now(), self.odom_frame, self.world_frame)
        except:
            pass

def main():
    rospy.init_node('tf_t265_world_odom_glue')
        
    if len(sys.argv) >= 4:
        world_frame = sys.argv[1]
        t265_world_frame = sys.argv[2]

        odom_frame = sys.argv[3]
        t265_odom_frame = sys.argv[4]

        odom_topic = sys.argv[5]
    else:
        try:
            world_frame = rospy.get_param('~world_frame')
            t265_world_frame = rospy.get_param('~t265_world_frame')

            odom_frame = rospy.get_param('~odom_frame')
            t265_odom_frame = rospy.get_param('~t265_odom_frame')

            odom_topic = rospy.get_param('~odom_topic')
        except:
            rospy.logerr("Missing world and odom parameters")
            return       

    OdomGlue(world_frame, t265_world_frame, odom_frame, t265_odom_frame, odom_topic)
    rospy.spin()

if __name__ == '__main__':
    main()