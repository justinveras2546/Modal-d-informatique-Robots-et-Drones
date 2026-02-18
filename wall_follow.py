#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# Params PID
kp = 5
kd = 0.001
ki = 0.009
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

# Params mur
DESIRED_DISTANCE_RIGHT = 1.2
DESIRED_DISTANCE_LEFT = 0.7
VELOCITY = 4 
CAR_LENGTH = 0.50 
LOOKAHEADD= 0.9

class WallFollow:
    
    def __init__(self):
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        drive_topic_reel= '/vesc/ackermann_cmd_mux/input/navigation'
        self.prev_time= rospy.get_time()
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

    def getRange(self, data, angle):
        if angle < data.angle_min or angle > data.angle_max:
            return float('inf')

        # Index du scan
        index = int((angle - data.angle_min) / data.angle_increment)
        
        if index < 0 or index >= len(data.ranges):
            return float('inf')

        dist = data.ranges[index]
        
        if math.isinf(dist) or math.isnan(dist):
            return float('inf')
            
        return dist

    def pid_control(self, error, velocity):
        global integral, prev_error, kp, ki, kd
        
        time = rospy.get_time()
        dt = time - self.prev_time
        integral += error * dt
        
        # Calcul PID
        angle = kp*error + kd*(error - prev_error)/dt + ki*integral
        
        # Vitesse selon angle
        if abs(angle) <= 0.35: 
            velocity = 2
        elif abs(angle) <= 0.698:
            velocity = 0.7
        else:
            velocity = 0.8

        prev_error = error
        self.prev_time = time
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        return leftDist - DESIRED_DISTANCE_LEFT

    def lidar_callback(self, data):
        global VELOCITY
        theta = 67
        theta_rad = math.radians(theta) 
        angle_b = math.radians(90) 
        angle_a = math.radians(90 - theta) 

        b = self.getRange(data, angle_b)
        a = self.getRange(data, angle_a)

        # Angle alpha et projection
        alpha = np.arctan((a*np.cos(theta_rad)- b)/(a*np.sin(theta_rad)))
        Dt = b*np.cos(alpha)
        Dt1 = Dt + LOOKAHEADD*np.sin(alpha)
        
        self.pid_control(self.followLeft(data, Dt1), VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)