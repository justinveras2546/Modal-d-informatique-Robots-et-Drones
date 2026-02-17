#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 5
kd = 0.001
ki = 0.009
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.8 # meters
DESIRED_DISTANCE_LEFT = 0.7
VELOCITY = 4 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
LOOKAHEADD= 0.9

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        drive_topic_reel= '/vesc/ackermann_cmd_mux/input/navigation'
        self.prev_time= rospy.get_time()
        self.lidar_sub = rospy.Subscriber(  lidarscan_topic, LaserScan, self.lidar_callback) # Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        if angle < data.angle_min or angle > data.angle_max:
            return float('inf')

        # Formule correcte pour trouver l'index
        index = int((angle - data.angle_min) / data.angle_increment)
        
        # Protection contre les débordements de tableau
        if index < 0 or index >= len(data.ranges):
            return float('inf')

        dist = data.ranges[index]
        
        # Gestion des infinis ou NaNs
        if math.isinf(dist) or math.isnan(dist):
            return float('inf')
            
        return dist

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        time= rospy.get_time()
        dt= time - self.prev_time
        integral= integral+ error*dt
        angle = kp*error + kd*(error - prev_error)/dt + ki*integral
        #TODO: Use kp, ki & kd to implement a PID controller for 
        if abs(angle) <= 0.35:  # Si l'angle (gauche ou droite) est petit
            velocity = 2
        elif abs(angle) <= 0.698:
            velocity = 0.7
        else:
            velocity = 0.8
        prev_error = error
        self.prev_time= time
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        
        error = leftDist - DESIRED_DISTANCE_LEFT
        return error

    def lidar_callback(self, data):
        global VELOCITY
        theta= 67 # Angle entre les deux rayons
        theta_rad = math.radians(theta) 
        angle_b = math.radians(90) # 90 deg à gauche
        angle_a = math.radians(90 - theta) 

        b = self.getRange(data, angle_b)
        a = self.getRange(data, angle_a)

        alpha = np.arctan((a*np.cos(theta_rad)- b)/(a*np.sin(theta_rad)))
        Dt= b*np.cos(alpha)
        Dt1= Dt + LOOKAHEADD*np.sin(alpha)
        error = self.followLeft(data, Dt1) # replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
