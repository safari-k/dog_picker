#!/usr/bin/env python

"""
    tf_head_tracker.py - Version 1.0 2011-08-01
    
    Move the head to track a PointStamped target on the /target_point topic.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

#import roslib; roslib.load_manifest('pi_head_tracking_3d_part2')
import rospy
import tf
from std_msgs.msg import Float64
from dynamixel_controllers.srv import *
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import JointState
from math import radians, sqrt
import sys

""" A speed of exactly 0 has a special meaning for Dynamixel servos--namely, "move as fast as you can".
    This can have some very undesirable consequences since it is the complete opposite of what 0 normally
    means.  So we define a very small speed value to represent zero speed.
"""
ZERO_SPEED = 0.0001

class tfTracker():
    def __init__(self):
        rospy.init_node('tf_head_tracker')
        
        rospy.on_shutdown(self.shutdown)
        
        """ How fast should we update the servos? """
        self.rate = rospy.get_param('~rate', 10)
        r = rospy.Rate(self.rate)
        
        """ Joint speeds are given in radians per second """
        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.3)
        self.max_joint_speed = rospy.get_param('~max_joint_speed', 0.5)
        
        """ How far ahead or behind the target (in radians) should we aim for? """
        self.lead_target_angle = rospy.get_param('~lead_target_angle', 0.5)
        
        """ How long (in seconds) should we permit the target to be lost before re-centering the servos? """
        self.target_timeout = 3.0
        self.target_lost = False
        self.servos_centered = False

        """ Remap these in the launch file or command line if necessary """
        self.camera_link = 'kinect_link'
        self.head_pan_joint = 'dynamixel_AX12_17_joint'
        self.head_tilt_joint = 'dynamixel_AX12_6_joint'
        self.head_pan_link = 'dynamixel_AX12_17_link'
        self.head_tilt_link = 'dynamixel_AX12_6_link'
        
        self.dynamixels = rospy.get_param('dynamixels', '')
        
        """ The pan/tilt thresholds indicate how far (in meters) the ROI needs to be off-center
            before we make a movement. """
        self.pan_threshold = int(rospy.get_param('~pan_threshold', 0.01))
        self.tilt_threshold = int(rospy.get_param('~tilt_threshold', 0.01))
        
        """ The k_pan and k_tilt parameter determine how responsive the servo movements are.
            If these are set too high, oscillation can result. """
        self.k_pan = rospy.get_param('~k_pan', 1.5)
        self.k_tilt = rospy.get_param('~k_tilt', 1.5)
        
        """ Set limits on how far we can pan or tilt """
        self.max_pan = rospy.get_param('~max_pan', radians(145))
        self.min_pan = rospy.get_param('~min_pan', radians(-145))
        self.max_tilt = rospy.get_param('~max_tilt', radians(90))
        self.min_tilt = rospy.get_param('~min_tilt', radians(-90))
        
        self.servo_speed = dict()
        self.servo_position = dict()
        self.torque_enable = dict()

        """ Connect to the set_speed and torque_enable services for each servo.
            Also define a position publisher for each servo. """
        for name in sorted(self.dynamixels):
            try:
                controller = name.replace("_joint", "") + "_controller"

                # The set_speed services
                set_speed_service = '/' + controller + '/set_speed'
                rospy.wait_for_service(set_speed_service)  
                self.servo_speed[name] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent=True)
                
                # Initialize the servo speed to the default_joint_speed
                self.servo_speed[name](self.default_joint_speed)
                
                # Torque enable/disable control for each servo
                torque_service = '/' + controller + '/torque_enable'
                rospy.wait_for_service(torque_service) 
                self.torque_enable[name] = rospy.ServiceProxy(torque_service, TorqueEnable)
                
                # Start each servo in the disabled state so we can move them by hand
                self.torque_enable[name](False)

                # The position controllers
                self.servo_position[name] = rospy.Publisher('/' + controller + '/command', Float64)
            except:
                rospy.loginfo("Can't contact servo services!")
        
        rospy.loginfo("TF Tracker node started. Centering servos...")
        
        self.pan_position = 0
        self.tilt_position = 0
        self.pan_speed = ZERO_SPEED
        self.tilt_speed = ZERO_SPEED
        
        self.last_tilt_speed = 0
        self.last_pan_speed = 0

        """ Use a counter to detect when we have lost the target """
        self.tracking_seq = 0
        self.last_tracking_seq = -1
        self.target_lost_count = 0
        self.max_target_lost_count = self.rate * 5
    
        """ Center the pan and tilt servos at the start. """
        self.center_head_servos()
        
        """ Initialize tf listener """
        self.tf = tf.TransformListener()
        
        """ Make sure we can see the camera and head pan links """
        self.tf.waitForTransform(self.camera_link, self.head_pan_link, rospy.Time(), rospy.Duration(5.0))
        
        """ Wait also for the joint_states topic so we can track our own joint positions """
        rospy.wait_for_message('joint_states', JointState)
        self.joint_state = JointState()
        rospy.Subscriber('joint_states', JointState, self.update_joint_state)
        
        """ Subscribe to the target point topic """
        rospy.Subscriber('target_point', PointStamped, self.update_head_position)
                
        while not rospy.is_shutdown():
            if self.last_tracking_seq == self.tracking_seq:
                self.pan_speed = ZERO_SPEED
                self.tilt_speed = ZERO_SPEED
                self.target_lost_count += 1
            else:
                self.last_tracking_seq = self.tracking_seq
                self.target_lost_count = 0
                    
            if self.target_lost_count > self.max_target_lost_count:
                self.center_head_servos()

            else:               
                try:
                    """ Only update the pan speed if it differs from the last value """
                    if self.last_pan_speed != self.pan_speed:
                        self.servo_speed[self.head_pan_joint](self.pan_speed)
                        self.last_pan_speed = self.pan_speed
                    self.servo_position[self.head_pan_joint].publish(self.pan_position)
                except:
                    """ If we run into any exceptions, mometarily stop the head movement by setting
                    the target pan position to the current position. """
                    try:
                        current_pan_position = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
                        self.servo_position[self.head_pan_joint].publish(current_pan_position)
                        rospy.loginfo("Servo SetSpeed Exception!")
                        rospy.loginfo(sys.exc_info())
                    except:
                        pass
                                 
                try:
                    """ Only update the tilt speed if it differs from the last value """
                    if self.last_tilt_speed != self.tilt_speed:
                        self.servo_speed[self.head_tilt_joint](self.tilt_speed)
                        self.last_tilt_speed = self.tilt_speed
                    self.servo_position[self.head_tilt_joint].publish(self.tilt_position)
                except:
                    """ If we run into any exceptions, mometarily stop the head movement by setting
                    the target tilt position to the current position. """
                    try:
                        current_tilt_position = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
                        self.servo_position[self.head_tilt_joint].publish(current_tilt_position)
                        rospy.loginfo("Servo SetSpeed Exception!")
                        rospy.loginfo(sys.exc_info())
                    except:
                        pass
                                    
            r.sleep()
            
    def center_head_servos(self):
        try:
            self.servo_speed[self.head_pan_joint](self.default_joint_speed)
            self.servo_speed[self.head_tilt_joint](self.default_joint_speed)
            for i in range(3):
                self.servo_position[self.head_pan_joint].publish(0)
                self.servo_position[self.head_tilt_joint].publish(0)
                rospy.sleep(1)
        except:
            pass
        
    def update_joint_state(self, msg):
        self.joint_state = msg
    
    def update_head_position(self, target):
        """ We increment a tracking counter upon receiving a target message so we can use the counter to
            determine when we have lost the target. """
        self.tracking_seq += 1
        
        """ Project the target point onto the camera link.
            In case of tf exceptions, simply return without an update. """
        try:         
            self.tf.waitForTransform(self.camera_link, target.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
            camera_target = self.tf.transformPoint(self.camera_link, target)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
        
        """ The virtual camera image is in the y-z plane """
        pan = -camera_target.point.y
        tilt = -camera_target.point.z
        
        """ Compute the distance to the target in the x direction """
        distance = float(abs(camera_target.point.x))
        
        """ Convert the pan and tilt values from meters to radians by dividing by the distance to the target.  Since the Kinect is 
            blind to distance within 0.5 meters, check for an exception and use 0.5 meters as a fall back. """
        try:
            pan /= distance
            tilt /= distance
        except:
            pan /= 0.5
            tilt /= 0.5
                      
        """ Pan the camera only if the displacement of the target point exceeds the threshold """
        if abs(pan) > self.pan_threshold:
            """ Set the pan speed proportion to the horizontal displacement of the target """
            self.pan_speed = trunc(min(self.max_joint_speed, max(ZERO_SPEED, self.k_pan * abs(pan))), 2)
               
            """ Set the target position ahead or behind the current position """
            try:
                current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
            except:
                return
            if pan > 0:
                self.pan_position = max(self.min_pan, current_pan - self.lead_target_angle)
            else:
                self.pan_position = min(self.max_pan, current_pan + self.lead_target_angle)
        else:
            self.pan_speed = ZERO_SPEED
        
        """ Tilt the camera only if the displacement of the target point exceeds the threshold """
        if abs(tilt) > self.tilt_threshold:
            """ Set the pan speed proportion to the vertical displacement of the target """
            self.tilt_speed = trunc(min(self.max_joint_speed, max(ZERO_SPEED, self.k_tilt * abs(tilt))), 2)
            
            """ Set the target position ahead or behind the current position """
            try:
                current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
            except:
                return
            if tilt < 0:
                self.tilt_position = max(self.min_tilt, current_tilt - self.lead_target_angle)
            else:
                self.tilt_position = min(self.max_tilt, current_tilt + self.lead_target_angle)

        else:
            self.tilt_speed = ZERO_SPEED
        
    def shutdown(self):
        rospy.loginfo("Shutting down frame tracking node...")
        self.center_head_servos()
        
        # Relax all servos to give them a rest.
        for servo in self.dynamixels:
            self.torque_enable[servo](False)
        
def trunc(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

                   
if __name__ == '__main__':
    try:
        tracker = tfTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF tracking node is shut down.")



