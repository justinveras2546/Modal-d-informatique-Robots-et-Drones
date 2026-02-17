#!/usr/bin/env python
import rospy
import pandas as pd
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WaypointVisualizer:
    def __init__(self):
        rospy.init_node('waypoint_visualizer_node')

        # --- CONFIGURATION ---
        # Mettez le chemin ABSOLU vers votre fichier CSV
        self.csv_path = '/home/justin/catkin_ws/src/pure_pursuit/src/course.csv'
        self.marker_topic = '/waypoint_vis'
        self.frame_id = 'map' # Important : repère global

        # Publishers
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1, latch=True)

        # Chargement et publication
        self.load_and_publish()
        
        rospy.spin()

    def load_and_publish(self):
        rospy.loginfo(f"Chargement des waypoints depuis : {self.csv_path}")
        
        try:
            # Lecture du CSV (sans header, comme dans votre code pure pursuit)
            df = pd.read_csv(self.csv_path, header=None, names=['x', 'y', 'z', 'w'])
        except Exception as e:
            rospy.logerr(f"Impossible de lire le fichier : {e}")
            return

        # Création du Marker (LINE_STRIP pour une ligne continue)
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "raceline"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # --- STYLE DE LA LIGNE ---
        marker.scale.x = 0.1  # Épaisseur de la ligne (10 cm)
        
        # Couleur (Bleu Cyan ici)
        marker.color.a = 1.0 # Opacité
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.pose.orientation.w = 1.0

        # Ajout des points
        for index, row in df.iterrows():
            p = Point()
            p.x = row['x']
            p.y = row['y']
            p.z = 0.0 # On force Z à 0 pour être au sol
            marker.points.append(p)

        # Publication
        self.marker_pub.publish(marker)
        rospy.loginfo(f"Publié {len(marker.points)} waypoints sur le topic {self.marker_topic}")

if __name__ == '__main__':
    try:
        WaypointVisualizer()
    except rospy.ROSInterruptException:
        pass
