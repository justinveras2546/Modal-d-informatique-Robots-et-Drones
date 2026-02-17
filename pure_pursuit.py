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

        # --- PARAMETRES ---
        self.LOOKAHEAD_DISTANCE = 0.6
        self.VELOCITY = 3
        self.MAX_STEER = 0.4189 
        self.map_frame = 'map'
        
        # Topic de commande (souvent /drive ou /nav en simu)
        self.drive_topic = '/nav' 

        # --- OPTIMISATION PERFORMANCE ---
        # 1. On charge le CSV UNE SEULE FOIS au démarrage.
        # Plus jamais on ne touchera au disque dur pendant que le robot roule.
        csv_path = '/home/justin/catkin_ws/src/pure_pursuit/src/course.csv'
        try:
            self.df = pd.read_csv(csv_path, header=None, names=['x', 'y', 'z', 'w'])
            rospy.loginfo(f"Trajectoire chargée en mémoire : {len(self.df)} points.")
        except Exception as e:
            rospy.logerr(f"Erreur CSV : {e}")
            self.df = pd.DataFrame({'x': [0], 'y': [0]})

        # Publishers / Subscribers
        self.sub = rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/visual_goal', Marker, queue_size=1)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def transformation_repere(self, robot_x, robot_y, robot_theta, goal_x, goal_y):
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        c = np.cos(robot_theta)
        s = np.sin(robot_theta)
        local_x = dx * c + dy * s
        local_y = -dx * s + dy * c
        return local_x, local_y

    def markerPose(self, mark_x, mark_y):
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
        
        
        # 1. Lecture position
        robot_x = pose_msg.pose.pose.position.x
        robot_y = pose_msg.pose.pose.position.y
        orientation_q = pose_msg.pose.pose.orientation
        robot_theta = self.get_yaw_from_quaternion(orientation_q)

       #rospy.loginfo(f"Robot Pose: x={robot_x:.2f}, y={robot_y:.2f}, theta={robot_theta:.2f}")

        dx = self.df['x'] - robot_x
        dy = self.df['y'] - robot_y
        self.df['dist_sq'] = dx**2 + dy**2

        # 3. Filtrage : Points devant
        x_local = dx * np.cos(robot_theta) + dy * np.sin(robot_theta)
        
        # Masque logique
        masque_valide = (x_local > 0) & (self.df['dist_sq'] >= self.LOOKAHEAD_DISTANCE**2)
        candidats = self.df[masque_valide]

        if candidats.empty:
            if self.df.empty: return
            target_row = self.df.sort_values('dist_sq').iloc[0]
        else:
            # On prend le premier point valide trié par distance
            target_row = candidats.sort_values('dist_sq').iloc[0]

        goal_x = target_row['x']
        goal_y = target_row['y']

        # Visualisation
        self.markerPose(goal_x, goal_y)

        # 4. Transformation et Commande
        local_x, local_y = self.transformation_repere(robot_x, robot_y, robot_theta, goal_x, goal_y)

        L2 = local_x**2 + local_y**2
        if L2 == 0: return

        curvature = 2 * local_y / L2
        steering_angle = math.atan(curvature)

        # Limites & Vitesse
       # if steering_angle > self.MAX_STEER: steering_angle = self.MAX_STEER
        #elif steering_angle < -self.MAX_STEER: steering_angle = -self.MAX_STEER

        speed = self.VELOCITY
        if abs(steering_angle) > 0.35:
            if abs(steering_angle) < 0.7:
                speed = speed * 0.8
            elif abs(steering_angle) < 1.0:
                speed = speed * 0.6
            else:
                speed = speed * 0.5


        # Publication
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
