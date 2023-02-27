import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from rclpy.qos import qos_profile_sensor_data
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Twist
import time
from threading import Thread

stop_cmd = Twist()

class ConeNavigator(Node):

    def __init__(self):
        super().__init__('cone_visualizer')

        self.angular_speed = 0.2
        self.linear_speed = 0.22
        self.vel_msg = Twist()
        self.moving = False

        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1000)

        # Subscribtions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile= qos_profile_sensor_data # 5Hz
        )

        self.vel_publisher.publish(self.vel_msg) # Stop the robot

        self.get_logger().info('Cone navigator node initialized.')

    def clusterize_scan(self, coord, cluster_th, min_samples):
        # Return labeled scan and number of clusters found
        dbscan = DBSCAN(eps=cluster_th, min_samples=min_samples)    
        dbscan.fit(coord)                   # Fitting the model to the data
        labels = dbscan.labels_             # Extracting the labels and number of clusters
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        return labels, n_clusters

    def find_cluster_centers(self, ranges, angles, clusters, n_clusters):
        X = np.array([ranges, angles]).T
        centers = []
        for i in range(n_clusters):
            center = np.mean(X[clusters == i], axis=0)
            centers.append(center)
        return np.array(centers)
    
    def midpoint_polar(self, p1, p2):
        # Converting polar coordinates to cartesian coordinates
        x1, y1 = p1[0] * np.cos(p1[1]), p1[0] * np.sin(p1[1])
        x2, y2 = p2[0] * np.cos(p2[1]), p2[0] * np.sin(p2[1])

        # Calculating the midpoint
        midpoint = [(x1 + x2) / 2, (y1 + y2) / 2]

        # Calculating the distance and angle between the midpoint and the origin
        distance = np.sqrt(midpoint[0] ** 2 + midpoint[1] ** 2)
        angle = np.arctan2(midpoint[1], midpoint[0])

        return midpoint, distance, angle

    def turn(self, vel_msg, angle = 0, angular_speed = 0.2):
        vel_msg.linear.x = 0.0
        vel_msg.angular.z =  angular_speed if angle > 0 else -angular_speed
        self.vel_publisher.publish(vel_msg)
        print("\tturning...") 
        time.sleep(abs(angle/self.angular_speed))
        self.vel_msg.angular.z = 0.0
        self.vel_publisher.publish(self.vel_msg) # stop

    def move(self, vel_msg, length = 0.25):
        vel_msg.linear.x = self.linear_speed
        self.vel_publisher.publish(vel_msg)
        print("\tmoving forward...")         
        time.sleep(abs(length / self.linear_speed)) # letting the robot move
        vel_msg.linear.x = 0.0
        self.vel_publisher.publish(vel_msg) # stop

    def drive2point(self, target_dist, target_angle):
        # simple driving logic
        self.moving = True
        if (abs(target_angle) > np.deg2rad(4)):
            self.turn(self.vel_msg, target_angle)
            # self.moving = False
            # return 
        if (target_dist > 0.1):
            self.move(self.vel_msg, target_dist + 0.1)
        self.moving = False

    
    def scan_callback(self, msg, debug=True): 
        # Getting the ranges and angles from the message
        ranges = np.array(msg.ranges) # meters
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment) # radians

        # Only keep the first and last 60 degrees
        ranges = np.concatenate([ranges[0:60], ranges[-60:]])
        angles = np.concatenate([angles[0:60], angles[-60:]])

        # Masking the infinities
        mask = np.isfinite(ranges)
        ranges_m = ranges[mask]
        angles_m = angles[mask]

        # If not enough points are found, return
        if len(ranges_m) < 10:
            print('Not enough points found')
            return

        # Clusterizing the scan
        coord = np.array([ranges_m * np.cos(angles_m), ranges_m * np.sin(angles_m)]).T
        clusters, n_clusters = self.clusterize_scan(coord, cluster_th=0.1, min_samples=5)

        # Calculating the center of each cluster 
        cluster_centers = self.find_cluster_centers(ranges_m, angles_m, clusters, n_clusters)

        # Filter clusters that are too close to the robot
        cluster_centers = cluster_centers[cluster_centers[:, 0] > 0.2]

        if len(cluster_centers) < 2:
            print('Not enough clusters found, %i found' % len(cluster_centers))
            return
        
        # Sort the clusters by distance
        cluster_centers = cluster_centers[cluster_centers[:, 0].argsort()]
        
        # Calculating the distance and angle between the midpoint and the origin
        target_position, target_dist, target_angle = self.midpoint_polar(cluster_centers[0], cluster_centers[1])

        # if the robot is already moving, do not apply driving logic
        if  not self.moving: # if the robot is already moving, do not apply driving logic
            drive_thread = Thread(target=self.drive2point, args=(target_dist, target_angle))
            drive_thread.start()
        
        # Debugging
        if debug:
            print('Target position: x:%.2f y:%.2f' % (target_position[0], target_position[1]))
            print('Target distance: %.2f m' % target_dist)
            print('Target angle: %.2f degree' % np.rad2deg(target_angle))

def main(args=None):
    rclpy.init(args=args)
    cone_navigator = ConeNavigator()
    rclpy.spin(cone_navigator)
    cone_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
