#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
# TODO: import ROS msg types and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool


class Safety(object):
    """
    The class that handles emergency braking.

    """
    Odom = None
    Scan = None
    ttc_threshold = 1.6# seconds
    brake = None
    brake_bool= None
    def __init__(self):
        """
    One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

    One publisher should publish to the /brake_bool topic with a Bool message.

    You should also subscribe to the /scan topic to get the LaserScan messages and
    the /odom topic to get the current speed of the vehicle.

    The subscribers should use the provided odom_callback and scan_callback as callback methods

    NOTE that the x component of the linear velocity in odom is the speed
        """
        global brake
        global brake_bool
        #brake = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
        brake = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=10)
        #brake_bool = rospy.Publisher('/brake_bool', Bool, queue_size=10)
        brake_bool = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/active', Bool, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)   
        #rospy.Subscriber('/vesc/odom', Odometry, self.odom_callback)     
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.loginfo("ABS Node started.")
        self.speed = 0
        # TODO: create ROS subscribers and publishers.


    def odom_callback(self, odom_msg):
    
        global Odom
        Odom= odom_msg
        self.speed = Odom.twist.twist.linear.x

    """
     def scan_callback(self, scan_msg):
        global Scan
        Scan= scan_msg
        # TODO: calculate TTC
        ttc = float('inf')
        angle = Scan.angle_min
        for r in Scan.ranges:
            if r > 0:
                relative_speed = abs (self.speed * np.sin(angle))
                if relative_speed > 0:
                    ttc_candidate = r / relative_speed
                    if ttc_candidate < ttc:
                        ttc = ttc_candidate
                        rospy.loginfo("TTC: {:.2f} seconds".format(ttc) +"ANGLE: {:.2f} rad".format(angle) + "Range: {:.2f} seconds".format(r) )
                        
            angle += Scan.angle_increment
    """

    def scan_callback(self, scan_msg):
        global Scan
        Scan = scan_msg
        ranges= list(Scan.ranges)
        indice =len(ranges)//2
        r= ranges[indice]
        if self.speed > 0:
            ttc = r / self.speed
            rospy.loginfo("TTC: {:.2f} seconds".format(ttc) + "Range: {:.2f} seconds".format(r)+ "vitesse: {:.2f} ms".format(self.speed))
            if ttc < self.ttc_threshold:
                rospy.loginfo("Emergency Brake Activated! TTC: {:.2f} seconds".format(ttc))
                brake_msg = AckermannDriveStamped()
                brake_msg.drive.speed = 0.0
                brake_bool_msg = Bool()
                brake_bool_msg.data = True
                # Publish brake message
                brake.publish(brake_msg)
                brake_bool.publish(brake_bool_msg)
            """
            else:
                brake_bool_msg = Bool()
                brake_bool_msg.data = False
                brake_bool.publish(brake_bool_msg)
                # TODO: publish brake message and publish controller bool
            """
        
    	
   


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
