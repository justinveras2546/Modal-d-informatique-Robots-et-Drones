#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ESE 680
RRT assignment - Solution Intégrée avec Pure Pursuit Local
Author: Hongrui Zheng / Modifié pour F1TENTH
"""
import numpy as np
import math
import random
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry 
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class Node(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRT(object):
    def __init__(self):
        rospy.init_node('rrt_node')

        # --- PARAMETRES ---
        self.LOOKAHEAD_DIST = 0.7 # Pure Pursuit
        self.VELOCITY = 1.5       # Vitesse
        
        # Paramètres RRT
        self.step_size = 0.3
        self.max_iter = 300       # Itérations max (Performance)
        self.goal_sample_rate = 0.20 # 20% vers le but
        
        # Grille locale (6x6m centrée)
        self.grid_res = 0.05
        self.grid_size = 240     
        self.grid_center = 120   
        
        # Grille occupation (0=Libre, 1=Mur)
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        # Topics
        scan_topic = rospy.get_param('~scan_topic', '/scan')
        drive_topic = rospy.get_param('~drive_topic', '/nav')

        # Subs
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        
        # Timer pour boucle de planning stable
        rospy.Timer(rospy.Duration(0.05), self.plan_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        # --- VISUALISATION ---
        self.viz_path_pub = rospy.Publisher('/visual/rrt_path', Path, queue_size=1)
        self.viz_target_pub = rospy.Publisher('/visual/target_point', Marker, queue_size=1)
        
        self.local_goal = None

    def publish_visuals(self, path_nodes, target_node):
        """ Visu chemin (bleu) et cible (rouge) """
        timestamp = rospy.Time.now()

        # 1. VISU CHEMIN
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = timestamp

        for node in path_nodes:
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = node.x
            pose.pose.position.y = node.y
            pose.pose.orientation.w = 1.0 # Neutre
            path_msg.poses.append(pose)
        
        self.viz_path_pub.publish(path_msg)

        # 2. VISU CIBLE
        if target_node is not None:
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = timestamp
            marker.ns = "pure_pursuit"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = target_node.x
            marker.pose.position.y = target_node.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2
            
            marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
            
            self.viz_target_pub.publish(marker)

    def scan_callback(self, scan_msg):
        """ Grille locale via LiDAR (Robot au milieu) """
        # 1. Polaire vers Cartésien
        raw_ranges = np.array(scan_msg.ranges)
        angles = np.arange(scan_msg.angle_min, 
                           scan_msg.angle_min + len(raw_ranges)*scan_msg.angle_increment, 
                           scan_msg.angle_increment)
        if len(angles) > len(raw_ranges): angles = angles[:len(raw_ranges)]

        # --- CIBLE : POINT LE PLUS LOIN ---
        # Cône avant
        front_min_angle = -np.pi / 3
        front_max_angle =  np.pi / 3
        angle_mask = (angles >= front_min_angle) & (angles <= front_max_angle)
        
        front_ranges = raw_ranges[angle_mask]
        front_angles = angles[angle_mask]
        
        # Gestion infinis
        sensor_max_range = 10.0 
        front_ranges[np.isinf(front_ranges)] = sensor_max_range
        front_ranges[np.isnan(front_ranges)] = 0.0

        if len(front_ranges) > 0:
            # Index du max
            max_local_idx = np.argmax(front_ranges)
            target_dist = front_ranges[max_local_idx]
            target_angle = front_angles[max_local_idx]
            
            # Astuce : 99% du trajet pour éviter d'être DANS le mur
            safe_dist = target_dist * 0.99
            goal_x = safe_dist * np.cos(target_angle)
            goal_y = safe_dist * np.sin(target_angle)
            self.goal_node = Node(goal_x, goal_y)
        else:
            self.goal_node = Node(1.0, 0.0)

        # --- CONSTRUCTION DE LA GRILLE ---
        self.local_goal = self.goal_node
        ranges = raw_ranges
        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)

        # Mètres vers indices grille
        ix = (x_local / self.grid_res + self.grid_center).astype(int)
        iy = (y_local / self.grid_res + self.grid_center).astype(int)

        self.occupancy_grid.fill(0)
        
        # Filtre limites grille
        mask = (ix >= 0) & (ix < self.grid_size) & (iy >= 0) & (iy < self.grid_size)
        valid_ix = ix[mask]; valid_iy = iy[mask]

        self.occupancy_grid[valid_iy, valid_ix] = 1 # Marque obstacles

        """
        # --- 4. DÉFINITION DU BUT (GOAL) ---
        clean_ranges = np.array(scan_msg.ranges)
        clean_ranges[np.isnan(clean_ranges)] = 0
        max_idx = np.argmax(clean_ranges)
        max_dist = clean_ranges[max_idx]
        max_angle = angles[max_idx]
        margin = 0.5 
        target_dist = max(0.0, max_dist - margin)
        goal_x = target_dist * np.cos(max_angle)
        goal_y = target_dist * np.sin(max_angle)
        self.local_goal = Node(goal_x, goal_y)
        """

    def plan_callback(self, event):
        """ Boucle RRT + Pursuit """
        start_node = Node(0.0, 0.0)
    
        # Raccourci ligne droite
        if self.local_goal and not self.check_collision(start_node, self.local_goal):
            path = [start_node, self.local_goal]
            self.run_pure_pursuit(path)
            return
        
        # ALGORITHME RRT
        start_node = Node(0.0, 0.0)
        self.node_list = [start_node]
        path = []
        
        for i in range(self.max_iter):
            rnd_node = self.sample() # A. Sample
            nearest_ind = self.nearest(self.node_list, rnd_node) # B. Nearest
            nearest_node = self.node_list[nearest_ind]
            new_node = self.steer(nearest_node, rnd_node) # C. Steer
            
            if not self.check_collision(nearest_node, new_node): # D. Check Collision
                self.node_list.append(new_node)
                if self.is_goal(new_node): # E. Check Goal
                    path = self.find_path(self.node_list, new_node)
                    break
        
        # Fallback : noeud le plus loin si pas de path
        if not path:
             best_node = max(self.node_list, key=lambda n: n.x)
             path = self.find_path(self.node_list, best_node)

        if path:
            self.run_pure_pursuit(path)

    def sample(self):
        """ Sample hasard ou but """
        if random.random() > self.goal_sample_rate:
            # Box locale (favorise l'avant)
            rand_x = random.uniform(0.0, 3.0) 
            rand_y = random.uniform(-1, 1)
            return Node(rand_x, rand_y)
        else:
            return self.local_goal

    def nearest(self, tree, sampled_node):
        """ Index du noeud le plus proche """
        dists = [(node.x - sampled_node.x)**2 + (node.y - sampled_node.y)**2 for node in tree]
        return dists.index(min(dists))

    def steer(self, from_node, to_node):
        """ Nouveau noeud vers sample """
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * math.cos(theta)
        new_y = from_node.y + self.step_size * math.sin(theta)
        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        return new_node

    def check_collision(self, node1, node2):
        """ Check collision ligne via grille """
        dist = math.hypot(node2.x - node1.x, node2.y - node1.y)
        steps = int(dist / (self.grid_res / 2.0)) 
        if steps == 0: return False

        for i in range(steps + 1):
            t = i / float(steps)
            x = node1.x + (node2.x - node1.x) * t
            y = node1.y + (node2.y - node1.y) * t
            ix = int(x / self.grid_res + self.grid_center)
            iy = int(y / self.grid_res + self.grid_center)
            
            if ix < 0 or ix >= self.grid_size or iy < 0 or iy >= self.grid_size:
                return True # Hors map
            if self.occupancy_grid[iy, ix] == 1:
                return True 
        return False

    def is_goal(self, node):
        return math.hypot(node.x - self.local_goal.x, node.y - self.local_goal.y) < 0.3

    def find_path(self, tree, end_node):
        path = []
        curr = end_node
        while curr is not None:
            path.append(curr); curr = curr.parent
        return path[::-1] # Inverse liste

    def run_pure_pursuit(self, path):
        """ Suivi path RRT """
        target_point = None
        # Premier point assez loin (Lookahead)
        for node in path:
            if math.hypot(node.x, node.y) >= self.LOOKAHEAD_DIST:
                target_point = node; break

        # Chemin court : dernier point si > 10cm
        if target_point is None and len(path) > 0:
            last_node = path[-1]
            if math.hypot(last_node.x, last_node.y) > 0.1:
                target_point = last_node
            
        self.publish_visuals(path, target_point)
        if target_point is None: return

        # Calcul Pursuit
        L2 = target_point.x**2 + target_point.y**2
        if L2 < 0.001: return # Évite division par zéro

        curvature = 2 * target_point.y / L2
        steering_angle = math.atan(curvature)

        # Commande
        drive = AckermannDriveStamped()
        drive.header.stamp = rospy.Time.now()
        drive.header.frame_id = "base_link"
        drive.drive.steering_angle = steering_angle
        drive.drive.speed = self.VELOCITY
        
        # Réduction vitesse virages
        if abs(steering_angle) > 0.35:
            if abs(steering_angle) < 0.7:
                drive.drive.speed *= 0.8
            elif abs(steering_angle) < 1.0:
                drive.drive.speed *= 0.6
            else:
                drive.drive.speed *= 0.5
        
        self.drive_pub.publish(drive)

def main():
    try:
        rrt = RRT(); rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()