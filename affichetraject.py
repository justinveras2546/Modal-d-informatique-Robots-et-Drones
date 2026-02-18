#!/usr/bin/env python
import rospy
import pandas as pd
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WaypointVisualizer:
    def __init__(self):
        rospy.init_node('waypoint_visualizer_node')

        # Config des chemins et topics
        self.csv_path = '/home/justin/catkin_ws/src/pure_pursuit/src/track2.csv'
        self.marker_topic = '/waypoint_vis'
        self.frame_id = 'map' 

        # Publisher avec latch pour conserver l'affichage
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1, latch=True)

        self.load_and_publish()
        rospy.spin()

    def load_and_publish(self):
        rospy.loginfo(f"Chargement waypoints : {self.csv_path}")
        
        try:
            # Lecture du CSV via Pandas
            df = pd.read_csv(self.csv_path, header=None, names=['x', 'y', 'z', 'w'])
        except Exception as e:
            rospy.logerr(f"Erreur lecture : {e}")
            return

        # Config du Marker (ligne continue)
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "raceline"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Épaisseur (10cm) et couleur (Cyan)
        marker.scale.x = 0.1 
        marker.color.a = 1.0 
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.pose.orientation.w = 1.0

        # Remplissage des points (Z=0 pour le sol)
        for index, row in df.iterrows():
            p = Point()
            p.x = row['x']
            p.y = row['y']
            p.z = 0.0 
            marker.points.append(p)

        self.marker_pub.publish(marker)
        rospy.loginfo(f"Publié : {len(marker.points)} waypoints")

if __name__ == '__main__':
    try:
        WaypointVisualizer()
    except rospy.ROSInterruptException:
        pass