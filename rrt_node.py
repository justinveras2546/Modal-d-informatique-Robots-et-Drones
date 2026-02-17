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
from nav_msgs.msg import Odometry # On utilise l'Odom pour le timer souvent
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
        self.LOOKAHEAD_DIST = 0.5 # Pure Pursuit
        self.VELOCITY = 1.5       # Vitesse
        
        # RRT Params
        self.step_size = 0.3
        self.max_iter = 300       # Nombre max d'itérations (Performance !)
        self.goal_sample_rate = 0.20 # 20% du temps on vise directement le but
        
        # Grid Params (Map locale de 6m x 6m centree sur le robot)
        self.grid_res = 0.05
        self.grid_size = 240     # 120 * 0.05 = 6 mètres
        self.grid_center = 120     # Le robot est au milieu
        
        # La Grille d'occupation (0=Libre, 1=Obstacle)
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        # Topics
        scan_topic = rospy.get_param('~scan_topic', '/scan')
        drive_topic = rospy.get_param('~drive_topic', '/nav')

        # Subscribers
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        
        # On utilise un Timer pour la boucle de planification (plus stable que pf_callback)
        rospy.Timer(rospy.Duration(0.05), self.plan_callback)
        # ... (vos autres publishers) ...
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        # --- VISUALISATION ---
        self.viz_path_pub = rospy.Publisher('/visual/rrt_path', Path, queue_size=1)
        self.viz_target_pub = rospy.Publisher('/visual/target_point', Marker, queue_size=1)
        # Publishers
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        # Objectif Local (2 mètres devant le robot)
        self.local_goal = None

        

    def publish_visuals(self, path_nodes, target_node):
        """ Publie le chemin (Ligne Bleue) et la Cible (Sphère Rouge) """
        timestamp = rospy.Time.now()

        # 1. VISUALISER LE PATH (Ligne Bleue)
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = timestamp

        for node in path_nodes:
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = node.x
            pose.pose.position.y = node.y
            pose.pose.orientation.w = 1.0 # Orientation neutre
            path_msg.poses.append(pose)
        
        self.viz_path_pub.publish(path_msg)

        # 2. VISUALISER LE TARGET POINT (Boule Rouge)
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
            
            # Taille (20 cm)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Couleur (Rouge pétant)
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            self.viz_target_pub.publish(marker)




   

    def scan_callback(self, scan_msg):
        """
        Construit une Occupancy Grid locale à partir du LIDAR
        Le robot est toujours au centre de cette grille (index 60,60).
        """
        # 1. Conversion Polaire -> Cartésien
        raw_ranges = np.array(scan_msg.ranges)
        
        # Calcul des angles correspondant à chaque point
        angles = np.arange(scan_msg.angle_min, 
                           scan_msg.angle_min + len(raw_ranges)*scan_msg.angle_increment, 
                           scan_msg.angle_increment)
        if len(angles) > len(raw_ranges): angles = angles[:len(raw_ranges)]

        # --- LOGIQUE CIBLE : POINT LE PLUS LOIN A L'AVANT ---
        
        # A. Définir "L'avant" du véhicule (ex: -90° à +90° soit -PI/2 à +PI/2)
        # Tu peux réduire à -60°/+60° si tu veux éviter les murs latéraux
        front_min_angle = -np.pi / 3
        front_max_angle =  np.pi / 3
        
        # Masque pour ne garder que les indices qui sont dans le cône avant
        angle_mask = (angles >= front_min_angle) & (angles <= front_max_angle)
        
        # B. Nettoyage des données dans ce cône
        front_ranges = raw_ranges[angle_mask]
        front_angles = angles[angle_mask]
        
        # Gestion des 'inf' (infini) :
        # Si le lidar voit l'infini (tunnel vide), c'est l'endroit le plus loin !
        # On remplace 'inf' par la portée max du lidar (ex: 10m) pour qu'il soit choisi par argmax
        sensor_max_range = 10.0 
        front_ranges[np.isinf(front_ranges)] = sensor_max_range
        front_ranges[np.isnan(front_ranges)] = 0.0

        if len(front_ranges) > 0:
            # C. Trouver l'index du max absolu dans la zone avant
            max_local_idx = np.argmax(front_ranges)
            
            target_dist = front_ranges[max_local_idx]
            target_angle = front_angles[max_local_idx]
            
            # D. Coordonnées du Goal
            # ASTUCE CRITIQUE : On multiplie par 0.98 ou 0.99.
            # Cela place le point à 99% du trajet, juste "devant" le mur pixel.
            # Si on met 1.0, le goal tombe SUR le mur -> Le RRT échoue car "Goal in Collision".
            safe_dist = target_dist * 0.99
            
            goal_x = safe_dist * np.cos(target_angle)
            goal_y = safe_dist * np.sin(target_angle)
            
            self.goal_node = Node(goal_x, goal_y)
        else:
            # Sécurité si le lidar est aveugle ou tout est masqué
            self.goal_node = Node(1.0, 0.0) # Avance un peu tout droit

            # --- CONSTRUCTION DE LA GRILLE D'OCCUPATION ---
        self.local_goal=self.goal_node
        ranges = raw_ranges
        # Coordonnées dans le repère du robot (base_link)
        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)

        # 2. Conversion Mètres -> Indices Grille
        # ix = x / res + center
        ix = (x_local / self.grid_res + self.grid_center).astype(int)
        iy = (y_local / self.grid_res + self.grid_center).astype(int)

        # 3. Remplissage de la grille
        # On vide la grille précédente
        self.occupancy_grid.fill(0)
        
        # Filtrage pour ne garder que les points DANS la grille
        mask = (ix >= 0) & (ix < self.grid_size) & (iy >= 0) & (iy < self.grid_size)
        valid_ix = ix[mask]
        valid_iy = iy[mask]

        # On marque les obstacles (1)
        # Note : On inverse y et x pour l'indexation numpy [row, col] -> [y, x]
        self.occupancy_grid[valid_iy, valid_ix] = 1
        """
        # --- 4. DÉFINITION DU BUT (GOAL) ---
        # Stratégie : Le point valide le plus loin
        
        # a. Filtrage des données brutes
        # On ignore les 'inf' (souvent 0 ou max_range selon les lidars) pour trouver le vrai max physique
        clean_ranges = np.array(scan_msg.ranges)
        #clean_ranges[np.isinf(clean_ranges)] = 0 
        clean_ranges[np.isnan(clean_ranges)] = 0
        
        # b. Trouver l'index du point le plus loin
        # Optionnel : Restreindre à l'arc avant (-90° à +90°) pour ne pas faire demi-tour
        # Ici on prend tout le scan comme demandé.
        max_idx = np.argmax(clean_ranges)
        max_dist = clean_ranges[max_idx]
        max_angle = angles[max_idx] # Utilise le tableau 'angles' calculé plus haut
        
        # c. Calculer le Goal "Safe"
        # On recule de 'margin' mètres pour ne pas viser DANS le mur
        margin = 0.5 
        target_dist = max(0.0, max_dist - margin)
        
        goal_x = target_dist * np.cos(max_angle)
        goal_y = target_dist * np.sin(max_angle)
        
        # On sauvegarde ce but pour le planner
        self.local_goal = Node(goal_x, goal_y)
        """


    def plan_callback(self, event):
        """ Boucle principale RRT + Pure Pursuit """
        start_node = Node(0.0, 0.0)
    
        # --- RACCOURCI : Si la ligne droite est libre, on fonce ---
        if self.local_goal and not self.check_collision(start_node, self.local_goal):
            path = [start_node, self.local_goal]
            self.run_pure_pursuit(path)
            return # On saute le RRT
        
        # --- Sinon, on lance le RRT standard ---
        
        # 1. Point de départ : Le robot (0,0 local)
        start_node = Node(0.0, 0.0)
        self.node_list = [start_node]
        
        path = []
        
        # 2. ALGORITHME RRT
        for i in range(self.max_iter):
            # A. Sample
            rnd_node = self.sample()
            
            # B. Nearest
            nearest_ind = self.nearest(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]
            
            # C. Steer
            new_node = self.steer(nearest_node, rnd_node)
            
            # D. Collision Check & Add
            if not self.check_collision(nearest_node, new_node):
                self.node_list.append(new_node)
                
                # E. Check Goal
                if self.is_goal(new_node):
                    path = self.find_path(self.node_list, new_node)
                    break
        
        # Si aucun chemin trouvé, on essaie de suivre le meilleur noeud dispo (le plus avancé en X)
        if not path:
             # Fallback : prendre le noeud avec le plus grand X (le plus loin devant)
             best_node = max(self.node_list, key=lambda n: n.x)
             path = self.find_path(self.node_list, best_node)

        # 3. PURE PURSUIT SUR LE CHEMIN TROUVÉ
        if path:
            self.run_pure_pursuit(path)
        else:
            # Sécurité : Si vraiment rien, stop ou recule
            pass

    def sample(self):
        """ Retourne un point (x,y) au hasard ou le but """
        if random.random() > self.goal_sample_rate:
            # Random dans la boite locale (-1m a 3m en x, -2m a 2m en y)
            # On favorise l'avant (x > 0)
            rand_x = random.uniform(0.0, 3.0) 
            rand_y = random.uniform(-1, 1)
            return Node(rand_x, rand_y)
        else:
            return self.local_goal

    def nearest(self, tree, sampled_node):
        """ Retourne l'index du noeud le plus proche """
        dists = [(node.x - sampled_node.x)**2 + (node.y - sampled_node.y)**2 for node in tree]
        return dists.index(min(dists))

    def steer(self, from_node, to_node):
        """ Crée un nouveau noeud en direction du sample """
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * math.cos(theta)
        new_y = from_node.y + self.step_size * math.sin(theta)
        
        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        return new_node

    def check_collision(self, node1, node2):
        """ Vérifie la ligne entre 2 noeuds via la grille numpy """
        # Discrétisation simple
        dist = math.hypot(node2.x - node1.x, node2.y - node1.y)
        steps = int(dist / (self.grid_res / 2.0)) # check tous les demi-pixels
        
        if steps == 0: return False

        for i in range(steps + 1):
            t = i / float(steps)
            x = node1.x + (node2.x - node1.x) * t
            y = node1.y + (node2.y - node1.y) * t
            
            # Conversion grid
            ix = int(x / self.grid_res + self.grid_center)
            iy = int(y / self.grid_res + self.grid_center)
            
            # Check bounds
            if ix < 0 or ix >= self.grid_size or iy < 0 or iy >= self.grid_size:
                return True # Hors map = Risqué
            
            if self.occupancy_grid[iy, ix] == 1:
                return True # Collision
                
        return False

    def is_goal(self, node):
        dist = math.hypot(node.x - self.local_goal.x, node.y - self.local_goal.y)
        return dist < 0.3 # Tolérance de 30cm

    def find_path(self, tree, end_node):
        path = []
        curr = end_node
        while curr is not None:
            path.append(curr)
            curr = curr.parent
        return path[::-1] # Renverse la liste (Start -> End)

    def run_pure_pursuit(self, path):
        """ Suit le chemin généré par RRT """
        # 1. Trouver le point cible (Lookahead)
        target_point = None
        
        # On cherche le premier point qui est assez loin
        for node in path:
            dist = math.hypot(node.x, node.y)
            if dist >= self.LOOKAHEAD_DIST:
                target_point = node
                break
        




        # Fallback : Si le chemin est très court (ex: on est presque arrivé)
        # On prend le dernier point, MAIS seulement s'il n'est pas trop près (0,0)
        if target_point is None and len(path) > 0:
            last_node = path[-1]
            if math.hypot(last_node.x, last_node.y) > 0.1: # On ignore si < 10cm
                target_point = last_node
            
        self.publish_visuals(path, target_point)
        # Si toujours rien (ou si le seul point est (0,0)), on annule
        if target_point is None: 
            return

        # 2. Calcul Pure Pursuit
        L2 = target_point.x**2 + target_point.y**2
        
        # --- CORRECTION DU CRASH ICI ---
        # Si L2 est minuscule, on divise par zéro -> Crash
        if L2 < 0.001: 
            return

        curvature = 2 * target_point.y / L2
        steering_angle = math.atan(curvature)

        # 3. Commande
        drive = AckermannDriveStamped()
        drive.header.stamp = rospy.Time.now()
        drive.header.frame_id = "base_link"
        
        # Clamp Steering
        max_steer = 0.4
        #if steering_angle > max_steer: steering_angle = max_steer
        #if steering_angle < -max_steer: steering_angle = -max_steer
        
        drive.drive.steering_angle = steering_angle
        
        # Ralentir dans les virages
        drive.drive.speed = self.VELOCITY
        if abs(steering_angle) > 0.35:
            if abs(steering_angle) < 0.7:
                drive.drive.speed = drive.drive.speed * 0.8
            elif abs(steering_angle) < 1.0:
                drive.drive.speed = drive.drive.speed * 0.6
            else:
                drive.drive.speed = drive.drive.speed * 0.5
        
        self.drive_pub.publish(drive)

def main():
    try:
        rrt = RRT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()