import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from rclpy.qos import qos_profile_sensor_data
from sklearn.cluster import DBSCAN

class ConeVisualizer(Node):

    def __init__(self):
        super().__init__('cone_visualizer')
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile= qos_profile_sensor_data # 5Hz
        )

        self.get_logger().info('Cone visualizer node initialized.')

    def plot_scan(self, ranges, angles):
        plt.polar(angles, ranges, 'm.', markersize=1) # Plotting the scan
        plt.polar(0, 0, 'ks', markersize=10) # Plotting the robot
        plt.title('Laser scan')
        plt.pause(0.001)        
        plt.clf()                           # Clearing the plot

    def clusterize_scan(self, coord, cluster_th, min_samples):
        # Return labeled scan and number of clusters found
        dbscan = DBSCAN(eps=cluster_th, min_samples=min_samples)    
        dbscan.fit(coord)                   # Fitting the model to the data
        labels = dbscan.labels_             # Extracting the labels and number of clusters
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        return labels, n_clusters

    def scan_callback(self, msg, debug=True): 
        # Getting the ranges and angles from the message
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Masking the infinities
        mask = np.isfinite(ranges)
        ranges_m = ranges[mask]
        angles_m = angles[mask]

        # Converting to cartesian coordinates
        coord = np.array([ranges_m * np.cos(angles_m), ranges_m * np.sin(angles_m)]).T

        # Debugging
        if debug:
            print('Scan detected ', len(ranges_m), '(non-inf) ranges.')

        # clusters, n_clusters = self.clusterize_scan(coord, 0.1, 5)
        # print(n_clusters, ' clusters found')
        # print(clusters, ' labels')

        # Plotting
        self.plot_scan(ranges_m, angles_m)

def main(args=None):
    rclpy.init(args=args)

    cone_visualizer = ConeVisualizer()

    rclpy.spin(cone_visualizer)

    cone_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
