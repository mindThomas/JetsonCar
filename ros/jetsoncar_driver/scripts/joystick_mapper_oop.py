#!/usr/bin/env python  
import sys
import roslib
import rospy
import math
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
from jetsoncar_interfaces.msg import Setpoint
from jetsoncar_interfaces.srv import SetPID as srv_SetPID
from jetsoncar_interfaces.srv import SetRateLimits as srv_SetRateLimits

###  This is just an Object Oriented implementation of joystic_mapper.py  ###

class JoystickMapper:
    def __init__(self, joystick_topic, frequency):
        #ax_prev = [0]        
        self.ax = {} # prepare dictionary

        self.SetPID(0, 0, 0)
        self.SetRateLimits(999, 999)

        # Publish frequency can not be lower than 20 Hz
        self.frequency = max(frequency, 20)

        rospy.Subscriber(joystick_topic, sensor_msgs.msg.Joy, self.joystick_callback)
        self.pub_setpoint = rospy.Publisher('setpoint', Setpoint, queue_size=10)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / frequency), self.publish_setpoint)

    def joystick_callback(self, data):
        #global ax_prev
        raw = np.array(data.axes)
        #rospy.loginfo(raw)

        # Parse joystick values from PS3 Navigation controller    
        self.ax["x"]         = -raw[0]
        self.ax["y"]         = raw[1]

        self.ax["up"]        = abs(raw[8]) > 0
        self.ax["right"]     = abs(raw[9]) > 0
        self.ax["down"]      = abs(raw[10]) > 0
        self.ax["left"]      = abs(raw[11]) > 0

        self.ax["circle"]    = abs(raw[17]) > 0
        self.ax["cross"]     = abs(raw[18]) > 0

        self.ax["L1"]        = -raw[14]   
        self.ax["L2"]        = -raw[12]   

        #rospy.loginfo(ax)

        # if (len(ax_prev) == len(raw)):
        #     # diff = raw - ax_prev
        #     #diff = tuple(map(lambda i, j: abs(i - j), raw, ax_prev))
        #     diff = np.abs(raw - ax_prev)
        #     idx = np.argmax(diff)
        #     rospy.loginfo(idx)
        #ax_prev = raw 

    def publish_setpoint(self, event):
        msg = Setpoint()
        msg.angular_velocity = 0
        msg.steering_angle = 0

        if (len(self.ax) > 0):
            msg.angular_velocity = 20 * self.ax["L2"]
            msg.steering_angle = -self.ax["x"]

        try:
           self.pub_setpoint.publish(msg)
        except:
           pass    

    def SetPID(self, P, I, D):
        rospy.wait_for_service('/jetsoncar/set_pid')
        try:
            set_pid = rospy.ServiceProxy('/jetsoncar/set_pid', srv_SetPID)
            ret = set_pid(P, I, D)
            if (not ret):
                rospy.logerr("Could not disable PID controller")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def SetRateLimits(self, max_acceleration, max_deceleration):
        rospy.wait_for_service('/jetsoncar/set_rate_limits')
        try:
            set_rate_limits = rospy.ServiceProxy('/jetsoncar/set_rate_limits', srv_SetRateLimits)
            ret = set_rate_limits(max_acceleration, max_deceleration)        
            if (not ret):
                rospy.logerr("Could not disable rate limits")        
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)


def main():        
    rospy.init_node('joystick_mapper')
        
    if len(sys.argv) == 2:
        joystick_topic = sys.argv[1]
        frequency = 20
    elif len(sys.argv) >= 3:
        joystick_topic = sys.argv[1]
        frequency = float(sys.argv[2])
    else:
        try:
            joystick_topic = rospy.get_param('~joystick_topic')
            frequency = rospy.get_param('~frequency')
        except:
            rospy.logerr("Missing joystick_topic and/or frequency parameters")
            return       
    
    JoystickMapper(joystick_topic, frequency)
    
    rospy.spin()

if __name__ == '__main__':
    main()