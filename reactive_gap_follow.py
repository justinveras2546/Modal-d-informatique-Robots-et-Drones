#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        self.radius = 0.95  # radius of the bubble
        self.velocity = 2
        self.prev_steering_angle = 0.0
        self.prev_best_idx = 0
        self.sensitivity = 0.5 # Sensitivity factor for speed adjustment
        self.odom=None
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        drive_topic_reel= '/vesc/ackermann_cmd_mux/input/navigation'
        
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. """
        proc_ranges = np.array(ranges)
        
        # 1. Handle NaNs and Infs
        proc_ranges[np.isnan(proc_ranges)] = 0
        proc_ranges[np.isinf(proc_ranges)] = np.inf
        
        # 2. Cap high values (Don't set to 0!)
        # Any point further than 3m is treated as 3m free space
       #proc_ranges[proc_ranges > 3] = 3.0 
        
        return proc_ranges
    
    def find_max_gap(self, free_space_ranges):
        #Return the start index & end index of the max gap in free_space_ranges 
        # Mask where the range is NOT zero (0 = obstacle)
        # We are looking for the longest sequence of non-zeros
        masked = np.where(free_space_ranges > 0, 1, 0)
        
        # Determine lengths of consecutive 1s
        # This is a standard fast way to find gaps using numpy
        bounded = np.hstack(([0], masked, [0]))
        difs = np.diff(bounded)
        starts = np.where(difs == 1)[0]
        ends = np.where(difs == -1)[0]
        
        if len(starts) == 0:
            return 0, len(free_space_ranges) - 1 # No obstacles found?
            
        # Find the longest gap
        lengths = ends - starts
        max_idx = np.argmax(lengths)
        
        return starts[max_idx], ends[max_idx] - 1
    
    """

    def find_max_gap(self, free_space_ranges):
        
        ##Return the start index & end index of the max gap.
        ##Selection criteria: Sum of distances (depth) instead of just width.
        
        # 1. Créer un masque : 1 si espace libre, 0 si obstacle
        masked = np.where(free_space_ranges > 0, 1, 0)
        
        # 2. Trouver les débuts et fins des séquences de 1 (les gaps)
        bounded = np.hstack(([0], masked, [0]))
        difs = np.diff(bounded)
        starts = np.where(difs == 1)[0]
        ends = np.where(difs == -1)[0] # 'ends' est exclusif ici pour le slicing
        
        # Cas particulier : aucun gap trouvé
        if len(starts) == 0:
            return 0, len(free_space_ranges) - 1
            
        # 3. NOUVELLE LOGIQUE : Calculer la "masse" de chaque gap
        # On fait la somme des valeurs (distances) dans chaque intervalle identifié
        gap_scores = []
        for s, e in zip(starts, ends):
            # On somme les distances contenues dans l'intervalle [s : e]
            score = np.sum(free_space_ranges[s:e])
            gap_scores.append(score)
        
        # 4. Trouver l'index du gap avec le score (somme) le plus élevé
        max_idx = np.argmax(gap_scores)
        
        # Retourner start et end (en ajustant end pour être inclusif si nécessaire par la suite du code)
        return starts[max_idx], ends[max_idx] - 1
    """
    
    """
   #prendre celui au millieu
    def find_best_point(self, start_i, end_i, ranges):
        # Au lieu de viser le point le plus profond (risque de collision),
        # on vise le CENTRE du gap pour une sécurité maximale.
        best_idx = (start_i + end_i) // 2
        return best_idx
    """


    """
    def find_best_point(self, start_i, end_i, ranges):
        
      # Trouve le point le plus loin (max range) dans le quart (1/4) central du gap.
    
        # 1. Calculer la longueur totale du gap
        gap_len = end_i - start_i + 1
        
        # 2. Définir la largeur de la fenêtre de recherche (1/4 du gap)
        window_len = int(gap_len / 4)
        
        # Sécurité : Si le gap est minuscule, on regarde au moins 1 point
        if window_len < 1:
            window_len = 1
            
        # 3. Définir les bornes de la fenêtre (centrée sur le milieu du gap)
        mid_idx = (start_i + end_i) // 2
        
        # On recule de la moitié de la fenêtre pour centrer
        window_start = mid_idx - (window_len // 2)
        window_end = window_start + window_len
        
        # Clips de sécurité pour ne pas sortir du gap (indices)
        window_start = max(start_i, window_start)
        window_end = min(end_i + 1, window_end) # +1 pour le slicing Python
        
        # 4. Trouver le point le plus loin DANS cette fenêtre
        # On extrait uniquement la portion centrale des données LIDAR
        central_ranges = ranges[window_start : window_end]
        
        if len(central_ranges) == 0:
            return mid_idx # Fallback au milieu si erreur
            
        # np.argmax donne l'index RELATIF (0, 1, 2...) dans le petit tableau
        max_relative_idx = np.argmax(central_ranges)
        
        # 5. Convertir en index global (Index du scan complet)
        best_idx = window_start + max_relative_idx
        
        return best_idx
    """
    def find_best_point(self, start_i, end_i, ranges):
        """
        Stratégie 'Deep Center' :
        1. Regarde dans le quart central du gap.
        2. Filtre les points > 4m.
        3. Retourne l'index médian parmi ces points profonds.
        """
        # --- 1. Définir la fenêtre (Quart Central) ---
        gap_len = end_i - start_i + 1
        window_len = int(gap_len/1.1 )
        if window_len < 1: window_len = 1
        
        mid_idx = (start_i + end_i) // 2
        window_start = mid_idx - (window_len // 2)
        window_end = window_start + window_len
        
        # Sécurité : ne pas sortir des limites du gap
        window_start = max(start_i, window_start)
        window_end = min(end_i + 1, window_end)
        
        # Extraction des distances dans cette fenêtre centrale
        central_ranges = ranges[window_start : window_end]
        
        if len(central_ranges) == 0:
             return mid_idx # Sécurité vide

        # --- 2. Filtrage des points > 4 mètres ---
        # np.where retourne les indices LOCAUX (relatifs à window_start)
        indices_lointains = np.where(central_ranges > 2.5)[0]
        
        if len(indices_lointains) > 0:
            # CAS A : On a trouvé des points > 4m
            # On prend l'index du MILIEU de cette liste de points lointains
            # Cela nous centre par rapport à l'ouverture lointaine
            idx_median = len(indices_lointains) // 2
            local_best = indices_lointains[idx_median]
            best_idx = window_start + local_best
        else:
            # CAS B (Fallback) : Aucun point ne dépasse 4m (Virage serré ou mur proche)
            # On se rabat sur la stratégie précédente : viser le point le plus loin (max)
            # ou simplement le milieu géométrique pour rester safe.
            # Ici, je choisis le Max pour essayer de trouver l'ouverture quand même.
            local_best = np.argmax(central_ranges)
            best_idx = window_start + local_best

        if abs(best_idx - self.prev_best_idx) < 10 : # Si le point choisi change brutalement (saut de plus de 15 indices)
            best_idx = self.prev_best_idx # On garde l'ancien point pour éviter les oscillations
            
        self.prev_best_idx = best_idx # Mémoriser pour le prochain cycle
        return best_idx
    
    def publish_nav(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        
        # 1. Preprocess (Convert tuple to np.array here)
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        # 2. Find closest point (obstacle)  
        min_idx = np.argmin(proc_ranges)
        min_dist = proc_ranges[min_idx]
        
        min_angle = data.angle_min + min_idx * data.angle_increment
        if min_angle > math.radians(85) or min_angle < math.radians(85 - 360):
            steering_angle = 0
            speed= self.velocity
            self.publish_nav(steering_angle, speed)
            return  

        # 3. Create 'Bubble' (Eliminate points around the closest obstacle)
        # Optimization: Don't calculate dist for every single point (too slow).
        # We assume the bubble covers a certain angle.
        
        # Calculate the angle span of the bubble based on radius and distance
        if min_dist == 0: min_dist = 0.001 # Avoid division by zero
        
        # Arc length formula: angle = radius / radius_of_obstacle
        bubble_angle = math.atan(self.radius / min_dist)
        
        # Convert angle to index count
        bubble_idx_span = int(bubble_angle / data.angle_increment)
        
        # Set indices to 0 inside the bubble
        start_bubble = max(0, min_idx - bubble_idx_span)
        end_bubble = min(len(proc_ranges) - 1, min_idx + bubble_idx_span)
        
        proc_ranges[start_bubble : end_bubble+1] = 0

        # 4. Find max length gap 
        start_gap, end_gap = self.find_max_gap(proc_ranges)

        # 5. Find the best point in the gap 
        best_idx = self.find_best_point(start_gap, end_gap, proc_ranges)

        # 6. Publish Drive message
        self.steering_angle = 1.7*(data.angle_min + best_idx * data.angle_increment) 
        
       
        speed = self.velocity
        
        if abs(self.steering_angle) > 0.35: # Turn
            speed=speed*0.8
        
            if abs(self.steering_angle) < 0.5:
                speed = speed*0.8
            elif abs(self.steering_angle) < 0.8:
                speed = speed*0.7
            elif abs(self.steering_angle) < 1.2:
                speed = speed*0.5
            elif abs(self.steering_angle) > 1.5:
                speed = speed*0.45
            else:
                speed = speed*0.4
        

       # 1. Calculer la différence par rapport au dernier cycle (en valeur absolue)
        angle_diff = abs(self.steering_angle - self.prev_steering_angle)
        
        # 2. Mettre à jour l'angle précédent pour la prochaine fois
        self.prev_steering_angle = self.steering_angle
        
        # 3. Calculer le facteur de vitesse (entre 0.0 et 1.0)
        # Formule : 1 / (1 + x) permet une courbe douce qui ne devient jamais négative.
        # Si angle_diff est proche de 0 (stable), le facteur est proche de 1 (vitesse max).
        speed_factor = 1.0 / (1.0 + (self.sensitivity * angle_diff))
        
        # Alternative Linéaire (plus agressive, peut nécessiter un 'max' pour ne pas être < 0)
        # speed_factor = max(0.2, 1.0 - (sensitivity * angle_diff))
        
        # 4. Calcul de la vitesse finale
        speed= speed * speed_factor
        
        
        

        self.publish_nav(self.steering_angle, speed)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)