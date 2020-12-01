#!/usr/bin/env python  
import sys
import roslib
import rospy
import math
import tf
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node('tf_footprint_odom_glue')
        
    if len(sys.argv) >= 3:
        frame_from = sys.argv[1]
        frame_to = sys.argv[2]
    else:
        try:
            frame_from = rospy.get_param('~robot')
            frame_to = rospy.get_param('~sensor')
        except:
            rospy.logerr("Missing robot and sensor parameters")
            return       

    listener = tf.TransformListener()
    listener.waitForTransform(frame_from, '/footprint', rospy.Time(0), rospy.Duration(100))

    try:
        (trans,rot) = listener.lookupTransform(frame_from, '/footprint', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Could not lookup transform")    
        return

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = frame_to
    static_transformStamped.child_frame_id = "footprint"

    static_transformStamped.transform.translation.x = trans[0]
    static_transformStamped.transform.translation.y = trans[1]
    static_transformStamped.transform.translation.z = trans[2]

    #quat = tf.transformations.quaternion_from_euler(
    #           float(sys.argv[5]),float(sys.argv[6]),float(sys.argv[7]))
    static_transformStamped.transform.rotation.x = rot[0]
    static_transformStamped.transform.rotation.y = rot[1]
    static_transformStamped.transform.rotation.z = rot[2]
    static_transformStamped.transform.rotation.w = rot[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()

if __name__ == '__main__':
    main()