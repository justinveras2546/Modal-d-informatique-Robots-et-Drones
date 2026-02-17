#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ArtificialPotentialField:
    def __init__(self):
        # Initialisation du noeud
        rospy.init_node("apf_driver", anonymous=True)
        

        drive_topic = '/nav'  # Topic de commande pour VESC
        drive_topic_reel= '/vesc/ackermann_cmd_mux/input/navigation'
        # Subscribers / Publishers
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

        # --- PARAMETRES ---
        self.K_ATT = 0.9     # Force d'attraction (constante vers l'avant)
        self.K_REP = 1     # Gain de repulsion (sensibilite aux obstacles)
        self.D_REP = 1.5 # Distance de securite (metres)
        
        # Limites du robot
        self.MAX_SPEED = 2.2
        self.MIN_SPEED = 0.5
        self.MAX_STEERING_ANGLE = 1 # ~23 degres
        
        # Pour le lissage (Smoothing)
        self.last_steering_angle = 0.0
        self.alpha = 0.3  # Coefficient de lissage (0.2 = tres lisse, 1.0 = reactif)

    def get_repulsive_force(self, ranges, angles):
        """
        Calcule le vecteur de repulsion base sur les obstacles.
        """
        # Masque 1: On ignore les obstacles trop loin (hors zone d'influence)
    # Masque 2: On ignore les obstacles DERRIERE le robot (FOV de 180 seulement)
        fov_mask = (np.abs(angles) < np.radians(85)) 
        dist_mask = (ranges < self.D_REP+1) & (ranges > 0.05) # 0.05 pour eviter div/0
        
        valid_mask = fov_mask & dist_mask
        
        if not np.any(valid_mask):
            return np.array([0.0, 0.0])

        d = ranges[valid_mask]
        theta = angles[valid_mask]

        # Magnitude de la force : augmente exponentiellement quand on s'approche
        magnitude = 0.5 * self.K_REP * ((1.0 / d) - (1.0 / self.D_REP))**2

        # Direction : Opposee a l'obstacle
        force_x = np.sum(-magnitude * np.cos(theta))
        force_y = np.sum(-magnitude * np.sin(theta))

        return np.array([force_x, force_y])

    def lidar_callback(self, data):
        # 1. Preparation des donnees (Vectorisation Numpy)
        ranges = np.array(data.ranges)
        # Creation du tableau des angles correspondant a chaque point
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        
        # Ajustement taille (parfois le lidar renvoie 1 point de plus/moins)
        size = min(len(ranges), len(angles))
        ranges = ranges[:size]
        angles = angles[:size]

        # --- CORRECTION POUR PYTHON 2.7 / ANCIEN NUMPY ---
        # Au lieu de nan_to_num(posinf=...), on remplace manuellement :
        # 1. Remplacer les infinis par 10.0 (loin)
        ranges[np.isinf(ranges)] = 10.0
        # 2. Remplacer les NaN par 0.0 (erreur de lecture = ignor par le masque > 0.05 plus tard)
        ranges[np.isnan(ranges)] = 0.0
        # -------------------------------------------------

        # 2. Calcul des Forces
        # Force d'attraction : Toujours tout droit (dans le repere robot)
        f_att = np.array([self.K_ATT, 0.0])
        
        # Force de repulsion
        f_rep = self.get_repulsive_force(ranges, angles)

        # Resultante
        f_total = f_att + f_rep

        # 3. Calcul de la commande (Angle)
        target_angle = np.arctan2(f_total[1], f_total[0])

        # Saturation stricte
        target_angle = np.clip(target_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

        # 4. Lissage (Low Pass Filter)
        smooth_angle = (self.alpha * target_angle) + ((1 - self.alpha) * self.last_steering_angle)
        self.last_steering_angle = smooth_angle

        # 5. Calcul de la vitesse dynamique (Fonction lineaire)
        turn_ratio = abs(smooth_angle) / self.MAX_STEERING_ANGLE
        speed = self.MAX_SPEED * (1.0 - turn_ratio/1.5)
        
        # On garde une vitesse minimale pour ne pas caler
        speed = max(speed, self.MIN_SPEED)

        # 6. Publication
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = smooth_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        node = ArtificialPotentialField()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass