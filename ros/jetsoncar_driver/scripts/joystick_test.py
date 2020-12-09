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
        self.ax = {} # prepare dictionary
        self.setpoints = {}

        # Publish frequency can not be lower than 20 Hz
        self.frequency = max(frequency, 20)

        # Setpoints
        self.setpoints["target_angular_velocity"] = 8
        self.setpoints["angular_velocity"] = 0        
        
        self.steering_mode = "free" # free, cross, circle
        self.setpoints["target_free_steering_angle"] = 0        
        self.setpoints["target_cross_steering_angle"] = 0.8
        self.setpoints["target_circle_steering_angle"] = -0.8   
        self.setpoints["steering_angle"] = 0                

        # Create ROS Subscribers and Publishers
        rospy.Subscriber(joystick_topic, sensor_msgs.msg.Joy, self.joystick_callback)
        self.pub_setpoint = rospy.Publisher('setpoint', Setpoint, queue_size=10)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / frequency), self.publish_setpoint)

    def publish_setpoint(self, event):
        msg = Setpoint()
        msg.angular_velocity = self.setpoints["angular_velocity"]
        msg.steering_angle = self.setpoints["steering_angle"]

        try:
           self.pub_setpoint.publish(msg)
        except:
           pass   

    def joystick_callback(self, data):        
        raw = np.array(data.axes)
        #rospy.loginfo(raw)

        ax_prev = self.ax.copy()

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

        #rospy.loginfo(ax_prev)
        #rospy.loginfo(self.ax)        

        # Detect changes (button presses and releases)        
        if (len(ax_prev.keys()) == len(self.ax.keys())):
            diff = {}
            for key in self.ax.keys():
                diff[key] = int(self.ax[key]) - int(ax_prev[key])
            #diff = tuple(map(lambda i, j: abs(i - j), raw, ax_prev))
            #diff = np.abs(raw - ax_prev)
            #idx = np.argmax(diff)
            #rospy.loginfo(idx)     
            #print(diff)

            # Modify setpoints according to joystick changes
            # if self.steering_mode != "free" and (diff["right"] == 1 or diff["left"] == 1):
            #     self.setpoints["target_free_steering_angle"] = self.setpoints["steering_angle"]
            #     self.steering_mode = "free"

            # if (diff["right"] == 1) and (self.steering_mode == "free"):
            #     #rospy.loginfo("Right pressed")
            #     if self.setpoints["target_free_steering_angle"] > -1:
            #         self.setpoints["target_free_steering_angle"] = self.setpoints["target_free_steering_angle"] - 0.2
            #     rospy.loginfo("Steering angle: %2.1f" % self.setpoints["target_free_steering_angle"])
            # #elif (diff["right"] == -1):
            #     #rospy.loginfo("Right released")            

            # if (diff["left"] == 1) and (self.steering_mode == "free"):
            #     #rospy.loginfo("Left pressed")
            #     if self.setpoints["target_free_steering_angle"] < 1:
            #         self.setpoints["target_free_steering_angle"] = self.setpoints["target_free_steering_angle"] + 0.2
            #     rospy.loginfo("Steering angle: %2.1f" % self.setpoints["target_free_steering_angle"])
            # #elif (diff["left"] == -1):
            #     #rospy.loginfo("Left released") 
                 
            # if diff["up"] == 1:
            #     #rospy.loginfo("Up pressed")
            #     if self.setpoints["target_angular_velocity"] < 16:
            #         if self.setpoints["target_angular_velocity"] == -4:
            #             self.setpoints["target_angular_velocity"] = self.setpoints["target_angular_velocity"] + 8
            #         else:
            #             self.setpoints["target_angular_velocity"] = self.setpoints["target_angular_velocity"] + 1
            #     rospy.loginfo("Speed setpoint: %2.0f" % self.setpoints["target_angular_velocity"])
            # #elif (diff["up"] == -1):
            #     #rospy.loginfo("Up released")            

            # if diff["down"] == 1:
            #     #rospy.loginfo("Down pressed")
            #     if self.setpoints["target_angular_velocity"] > -16:
            #         if self.setpoints["target_angular_velocity"] == 4:
            #             self.setpoints["target_angular_velocity"] = self.setpoints["target_angular_velocity"] - 8
            #         else:
            #             self.setpoints["target_angular_velocity"] = self.setpoints["target_angular_velocity"] - 1
            #     rospy.loginfo("Speed setpoint: %2.0f" % self.setpoints["target_angular_velocity"])
            # #elif (diff["down"] == -1):
            #     #rospy.loginfo("Down released") 

            # if self.ax["L1"] > 0.5:
            #     if diff["cross"] == 1:
            #         self.setpoints["target_cross_steering_angle"] = self.setpoints["target_free_steering_angle"]
            #         rospy.loginfo("Set Cross steering setpoint to %2.1f" % self.setpoints["target_cross_steering_angle"])
            #     if diff["circle"] == 1:
            #         self.setpoints["target_circle_steering_angle"] = self.setpoints["target_free_steering_angle"]                    
            #         rospy.loginfo("Set Circle steering setpoint to %2.1f" % self.setpoints["target_circle_steering_angle"])
            # else:
            #     if diff["cross"] == 1:
            #         if self.steering_mode == "cross":
            #             self.steering_mode = "free"
            #             rospy.loginfo("Changed to Free steering mode")
            #         else:
            #             self.steering_mode = "cross"
            #             rospy.loginfo("Changed to Cross steering setpoint")

            #     if diff["circle"] == 1:
            #         if self.steering_mode == "circle":
            #             self.steering_mode = "free"
            #             rospy.loginfo("Changed to Free steering mode")
            #         else:
            #             self.steering_mode = "circle"
            #             rospy.loginfo("Changed to Circle steering setpoint")

            # if self.steering_mode == "free":
            #     self.setpoints["steering_angle"]  = self.setpoints["target_free_steering_angle"] 
            # elif self.steering_mode == "cross":
            #     self.setpoints["steering_angle"]  = self.setpoints["target_cross_steering_angle"] 
            # elif self.steering_mode == "circle":
            #     self.setpoints["steering_angle"]  = self.setpoints["target_circle_steering_angle"] 

            self.setpoints["steering_angle"] = -self.ax["x"]
            self.setpoints["target_angular_velocity"]  = 16 * self.ax["y"]

            if self.ax["L2"] > 0.8:
                self.setpoints["angular_velocity"] = self.setpoints["target_angular_velocity"]            
            else:
                self.setpoints["angular_velocity"] = 0
 


def main():        
    rospy.init_node('joystick_mapper')
        
    if len(sys.argv) == 2:
        joystick_topic = sys.argv[1]
        frequency = 20
    elif len(sys.argv) == 3:
        joystick_topic = sys.argv[1]
        frequency = float(sys.argv[2])
    else:
        try:
            joystick_topic = "joy"
            frequency = rospy.get_param('~frequency')
        except:
            rospy.logerr("Missing joystick_topic and/or frequency parameters")
            return       
    
    JoystickMapper(joystick_topic, frequency)
    
    rospy.spin()

if __name__ == '__main__':
    main()