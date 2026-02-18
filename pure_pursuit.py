#!/usr/bin/env python
import rospy
import math
import numpy as np
import pandas as pd
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

class PurePursuit(object):
    def __init__(self):
        rospy.init_node('pure_pursuit_node')

        # Paramètres de base
        self.LOOKAHEAD_DISTANCE = 0.6
        self.VELOCITY = 3
        self.MAX_STEER = 0.4189 
        self.map_frame = 'map'
        self.drive_topic = '/nav' 

        # Chargement CSV unique pour la performance
        csv_path = '/home/justin/catkin_ws/src/pure_pursuit/src/track2.csv'
        try:
            self.df = pd.read_csv(csv_path, header=None, names=['x', 'y', 'z', 'w'])
            rospy.loginfo(f"Trajectoire chargée : {len(self.df)} points.")
        except Exception as e:
            rospy.logerr(f"Erreur CSV : {e}")
            self.df = pd.DataFrame({'x': [0], 'y': [0]})

        # Pubs et Subs
        self.sub = rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/visual_goal', Marker, queue_size=1)

    def get_yaw_from_quaternion(self, q):
        # Conversion quaternion vers angle lacet
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def transformation_repere(self, robot_x, robot_y, robot_theta, goal_x, goal_y):
        # Passage du repère global au repère local robot
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        c = np.cos(robot_theta)
        s = np.sin(robot_theta)
        local_x = dx * c + dy * s
        local_y = -dx * s + dy * c
        return local_x, local_y

    def markerPose(self, mark_x, mark_y):
        # Marqueur pour visualiser la cible dans RViz
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = mark_x
        marker.pose.position.y = mark_y
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0
        self.marker_pub.publish(marker)

    def pose_callback(self, pose_msg):
        # 1. Lecture position et orientation
        robot_x = pose_msg.pose.pose.position.x
        robot_y = pose_msg.pose.pose.position.y
        orientation_q = pose_msg.pose.pose.orientation
        robot_theta = self.get_yaw_from_quaternion(orientation_q)

        # 2. Calcul des distances aux waypoints
        dx = self.df['x'] - robot_x
        dy = self.df['y'] - robot_y
        self.df['dist_sq'] = dx**2 + dy**2

        # 3. Recherche du point cible (devant et à bonne distance)
        x_local = dx * np.cos(robot_theta) + dy * np.sin(robot_theta)
        masque_valide = (x_local > 0) & (self.df['dist_sq'] >= self.LOOKAHEAD_DISTANCE**2)
        candidats = self.df[masque_valide]

        if candidats.empty:
            if self.df.empty: return
            target_row = self.df.sort_values('dist_sq').iloc[0]
        else:
            target_row = candidats.sort_values('dist_sq').iloc[0]

        goal_x = target_row['x']
        goal_y = target_row['y']
        self.markerPose(goal_x, goal_y)

        # 4. Calcul de la courbure et du braquage
        local_x, local_y = self.transformation_repere(robot_x, robot_y, robot_theta, goal_x, goal_y)
        L2 = local_x**2 + local_y**2
        if L2 == 0: return

        curvature = 2 * local_y / L2
        steering_angle = math.atan(curvature)

        # 5. Gestion de la vitesse selon l'angle
        speed = self.VELOCITY
        if abs(steering_angle) > 0.35:
            if abs(steering_angle) < 0.7:
                speed *= 0.8
            elif abs(steering_angle) < 1.0:
                speed *= 0.6
            else:
                speed *= 0.5

        # 6. Publication commande
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        PurePursuit()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass