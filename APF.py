#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ArtificialPotentialField:
    def __init__(self):
        # Initialisation du noeud
        rospy.init_node("apf_driver", anonymous=True)

        drive_topic = '/nav' 
        drive_topic_reel= '/vesc/ackermann_cmd_mux/input/navigation'
        
        # Subs et Pubs
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

        # --- PARAMETRES ---
        self.K_ATT = 0.9     # Force d'attraction vers l'avant
        self.K_REP = 1       # Gain de répulsion
        self.D_REP = 1.5     # Distance de sécurité
        
        # Limites robot
        self.MAX_SPEED = 2.2
        self.MIN_SPEED = 0.5
        self.MAX_STEERING_ANGLE = 1 
        
        # Lissage (Filtre passe-bas)
        self.last_steering_angle = 0.0
        self.alpha = 0.3  

    def get_repulsive_force(self, ranges, angles):
        """ Calcule le vecteur de répulsion des obstacles """
        # Masques : Champ de vision (85 deg) et distance
        fov_mask = (np.abs(angles) < np.radians(85)) 
        dist_mask = (ranges < self.D_REP+1) & (ranges > 0.05) 
        
        valid_mask = fov_mask & dist_mask
        
        if not np.any(valid_mask):
            return np.array([0.0, 0.0])

        d = ranges[valid_mask]
        theta = angles[valid_mask]

        # Magnitude : augmente exponentiellement près de l'obstacle
        magnitude = 0.5 * self.K_REP * ((1.0 / d) - (1.0 / self.D_REP))**2

        # Vecteur opposé à l'obstacle
        force_x = np.sum(-magnitude * np.cos(theta))
        force_y = np.sum(-magnitude * np.sin(theta))

        return np.array([force_x, force_y])

    def lidar_callback(self, data):
        # 1. Préparation des vecteurs
        ranges = np.array(data.ranges)
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        
        size = min(len(ranges), len(angles))
        ranges = ranges[:size]
        angles = angles[:size]

        # Gestion des Infinis et NaN
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 0.0

        # 2. Calcul des Forces
        # Attraction : toujours devant
        f_att = np.array([self.K_ATT, 0.0])
        f_rep = self.get_repulsive_force(ranges, angles)

        # Résultante
        f_total = f_att + f_rep

        # 3. Calcul angle cible
        target_angle = np.arctan2(f_total[1], f_total[0])
        target_angle = np.clip(target_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

        # 4. Lissage commande
        smooth_angle = (self.alpha * target_angle) + ((1 - self.alpha) * self.last_steering_angle)
        self.last_steering_angle = smooth_angle

        # 5. Vitesse dynamique (linéaire selon l'angle)
        turn_ratio = abs(smooth_angle) / self.MAX_STEERING_ANGLE
        speed = self.MAX_SPEED * (1.0 - turn_ratio/1.5)
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