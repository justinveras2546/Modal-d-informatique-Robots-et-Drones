#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# Imports ROS
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class reactive_follow_gap:
    def __init__(self):
        # Topics, abonnements et paramètres
        self.radius = 0.95  # Rayon de la bulle
        self.velocity = 2
        self.prev_steering_angle = 0.0
        self.prev_best_idx = 0
        self.sensitivity = 0.5 # Sensibilité pour l'ajustement de vitesse
        self.odom=None
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        drive_topic_reel= '/vesc/ackermann_cmd_mux/input/navigation'
        
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        
    def preprocess_lidar(self, ranges):
        """ Prétraitement du scan LiDAR """
        proc_ranges = np.array(ranges)
        
        # 1. Gère NaNs et Infs
        proc_ranges[np.isnan(proc_ranges)] = 0
        proc_ranges[np.isinf(proc_ranges)] = np.inf
        
        # 2. Plafonner les valeurs hautes (optionnel)
        # proc_ranges[proc_ranges > 3] = 3.0 
        
        return proc_ranges
    
    def find_max_gap(self, free_space_ranges):
        # Retourne début et fin du plus gros gap
        # Masque les obstacles (0)
        masked = np.where(free_space_ranges > 0, 1, 0)
        
        # Trouve les séquences de 1 consécutives
        bounded = np.hstack(([0], masked, [0]))
        difs = np.diff(bounded)
        starts = np.where(difs == 1)[0]
        ends = np.where(difs == -1)[0]
        
        if len(starts) == 0:
            return 0, len(free_space_ranges) - 1 # Pas d'obstacle
            
        # Cherche le gap le plus large
        lengths = ends - starts
        max_idx = np.argmax(lengths)
        
        return starts[max_idx], ends[max_idx] - 1
    
    """
    def find_max_gap(self, free_space_ranges):
        ## Critère : Somme des distances (profondeur) au lieu de largeur.
        
        masked = np.where(free_space_ranges > 0, 1, 0)
        bounded = np.hstack(([0], masked, [0]))
        difs = np.diff(bounded)
        starts = np.where(difs == 1)[0]
        ends = np.where(difs == -1)[0]
        
        if len(starts) == 0:
            return 0, len(free_space_ranges) - 1
            
        # Calcul du score par profondeur (somme des distances)
        gap_scores = []
        for s, e in zip(starts, ends):
            score = np.sum(free_space_ranges[s:e])
            gap_scores.append(score)
        
        max_idx = np.argmax(gap_scores)
        return starts[max_idx], ends[max_idx] - 1
    """
    
    """
    def find_best_point(self, start_i, end_i, ranges):
        # Vise le milieu du gap pour la sécurité.
        best_idx = (start_i + end_i) // 2
        return best_idx
    """

    """
    def find_best_point(self, start_i, end_i, ranges):
      # Vise le point le plus loin dans le quart central du gap.
    
        gap_len = end_i - start_i + 1
        window_len = int(gap_len / 4)
        if window_len < 1: window_len = 1
            
        mid_idx = (start_i + end_i) // 2
        window_start = mid_idx - (window_len // 2)
        window_end = window_start + window_len
        
        # Limites de sécurité
        window_start = max(start_i, window_start)
        window_end = min(end_i + 1, window_end)
        
        # Extraction portion centrale
        central_ranges = ranges[window_start : window_end]
        if len(central_ranges) == 0: return mid_idx
            
        max_relative_idx = np.argmax(central_ranges)
        return window_start + max_relative_idx
    """

    def find_best_point(self, start_i, end_i, ranges):
        """
        Stratégie 'Deep Center' :
        1. Fenêtre au quart central.
        2. Filtre points > 2.5m.
        3. Index médian des points profonds.
        """
        # 1. Fenêtre centrale
        gap_len = end_i - start_i + 1
        window_len = int(gap_len/1.1 )
        if window_len < 1: window_len = 1
        
        mid_idx = (start_i + end_i) // 2
        window_start = mid_idx - (window_len // 2)
        window_end = window_start + window_len
        
        window_start = max(start_i, window_start)
        window_end = min(end_i + 1, window_end)
        
        central_ranges = ranges[window_start : window_end]
        
        if len(central_ranges) == 0: return mid_idx 

        # 2. Filtrage profondeur (> 2.5m)
        indices_lointains = np.where(central_ranges > 2.5)[0]
        
        if len(indices_lointains) > 0:
            # Cas A : Milieu de l'ouverture lointaine
            idx_median = len(indices_lointains) // 2
            local_best = indices_lointains[idx_median]
            best_idx = window_start + local_best
        else:
            # Cas B : Vise le max local
            local_best = np.argmax(central_ranges)
            best_idx = window_start + local_best

        # Filtre anti-oscillation (saut brutal d'index)
        if abs(best_idx - self.prev_best_idx) < 10 : 
            best_idx = self.prev_best_idx 
            
        self.prev_best_idx = best_idx 
        return best_idx
    
    def publish_nav(self, steering_angle, speed):
        # Envoi commande Ackermann
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        # 1. Prétraitement
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        # 2. Point le plus proche (obstacle)
        min_idx = np.argmin(proc_ranges)
        min_dist = proc_ranges[min_idx]
        
        # Ignore si hors champ frontal (85 deg)
        min_angle = data.angle_min + min_idx * data.angle_increment
        if min_angle > math.radians(85) or min_angle < math.radians(85 - 360):
            self.publish_nav(0, self.velocity)
            return  

        # 3. Création de la bulle (élimine points autour de l'obstacle)
        if min_dist == 0: min_dist = 0.001 
        
        # Angle de la bulle selon rayon et distance
        bubble_angle = math.atan(self.radius / min_dist)
        bubble_idx_span = int(bubble_angle / data.angle_increment)
        
        # Masquage (mise à 0) dans la bulle
        start_bubble = max(0, min_idx - bubble_idx_span)
        end_bubble = min(len(proc_ranges) - 1, min_idx + bubble_idx_span)
        proc_ranges[start_bubble : end_bubble+1] = 0

        # 4. Cherche le plus grand gap
        start_gap, end_gap = self.find_max_gap(proc_ranges)

        # 5. Cherche le meilleur point cible
        best_idx = self.find_best_point(start_gap, end_gap, proc_ranges)

        # 6. Calcul angle et vitesse
        self.steering_angle = 1.7*(data.angle_min + best_idx * data.angle_increment) 
        speed = self.velocity
        
        # Réduction de vitesse en virage
        if abs(self.steering_angle) > 0.35: 
            speed=speed*0.8
            if abs(self.steering_angle) < 0.5:
                speed *= 0.8
            elif abs(self.steering_angle) < 0.8:
                speed *= 0.7
            elif abs(self.steering_angle) < 1.2:
                speed *= 0.5
            elif abs(self.steering_angle) > 1.5:
                speed *= 0.45
            else:
                speed *= 0.4
        
        # Facteur de vitesse selon l'écart d'angle (lissage)
        angle_diff = abs(self.steering_angle - self.prev_steering_angle)
        self.prev_steering_angle = self.steering_angle
        
        # Courbe de vitesse douce (1 / (1 + x))
        speed_factor = 1.0 / (1.0 + (self.sensitivity * angle_diff))
        speed = speed * speed_factor

        self.publish_nav(self.steering_angle, speed)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)