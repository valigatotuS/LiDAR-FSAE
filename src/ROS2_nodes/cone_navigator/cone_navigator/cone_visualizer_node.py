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
        plt.polar(angles, ranges, 'm.', markersize=1)   # Plotting the scan
        plt.polar(0, 0, 'ks', markersize=10)            # Plotting the robot
        plt.title('Laser scan')
        plt.pause(0.001)        
        plt.clf()                                       # Clearing the plot

    def plot_clusters(self, ranges, angles, clusters, n_clusters):
        plt.polar(angles, ranges, 'k.', markersize=3)   # Plotting the scan
        plt.polar(0, 0, 'ks', markersize=10)            # Plotting the robot
        colors = ['c', 'r', 'b', 'g', 'm', 'y', 'k', 'w', 'orange', 'purple', 'pink', 'brown', 'gray', 'olive', 'cyan', 'lightblue', 'lightgreen', 'tan', 'teal', 'lavender', 'turquoise', 'darkgreen', 'tan', 'salmon', 'gold', 'darkred', 'darkblue']

        # Plotting each cluster
        for i in range(n_clusters):
            print("cluster angles:", angles[clusters == i])
            plt.polar(angles[clusters == i], ranges[clusters == i], '.', markersize=1.5, c=colors[i], label=f'Cluster {i}')

    def clusterize_scan(self, coord, cluster_th, min_samples):
        # Return labeled scan and number of clusters found
        dbscan = DBSCAN(eps=cluster_th, min_samples=min_samples)    
        dbscan.fit(coord)                   # Fitting the model to the data
        labels = dbscan.labels_             # Extracting the labels and number of clusters
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        return labels, n_clusters
    
    def cirlce_fitting(self, ranges, angles, clusters, n_clusters):
        # -- TO DO --

        # getting the start and end index of each cluster
        # getting the x and y coordinates of each cluster
        # calculating the center of each cluster
        # calculating the center of each cluster
        # calculating the radius of each cluster
        # calculating the radius of each cluster
        # plotting circle on cartesian plot
        pass

    def find_cluster_centers(self, ranges, angles, clusters, n_clusters):
        X = np.array([ranges, angles]).T
        centers = []
        for i in range(n_clusters):
            center = np.mean(X[clusters == i], axis=0)
            centers.append(center)
            # calculating the distance between the origin (0,0) and the center (angle, distance) of the cluster



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
    
    def scan_callback(self, msg, debug=True): 
        # Getting the ranges and angles from the message
        ranges = np.array(msg.ranges) # meters
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment) # radians
        print(ranges, '###', angles)

        # Close view of the scan (120 degrees)
        ranges_f = np.concatenate([ranges[0:60], ranges[-60:]])
        angles_f = np.concatenate([angles[0:60], angles[-60:]])

        # Masking the infinities & zeros
        mask = np.isfinite(ranges_f) & (ranges_f > 0.01)
        ranges_m = ranges_f[mask]
        angles_m = angles_f[mask]

        mask2 = np.isfinite(ranges_f)
        ranges_mf = ranges_f[mask2]
        angles_mf = angles_f[mask2]

        # Clusterizing the scan
        coord = np.array([ranges_m * np.cos(angles_m), ranges_m * np.sin(angles_m)]).T
        clusters, n_clusters = self.clusterize_scan(coord, cluster_th=0.15, min_samples=5)
        print(n_clusters, ' clusters found')
        print(clusters, ' labels')

        coord_f = np.array([ranges_mf * np.cos(angles_mf), ranges_mf * np.sin(angles_mf)]).T
        clusters_f, n_clusters_f = self.clusterize_scan(coord_f, cluster_th=0.15, min_samples=5)
        print(n_clusters_f, 'valid clusters found')
        
        # Calculating the center of each cluster and sorting them by distance
        cluster_centers = self.find_cluster_centers(ranges_m, angles_m, clusters, n_clusters)
        cluster_centers = cluster_centers[cluster_centers[:, 0].argsort()]

        cluster_centers_f = self.find_cluster_centers(ranges_mf, angles_mf, clusters_f, n_clusters_f)
        cluster_centers_f = cluster_centers_f[cluster_centers_f[:, 0].argsort()]
        
        # Calculating the distance and angle between the midpoint and the origin
        target_position, target_dist, target_angle = [0, 0], 0, 0
        if cluster_centers_f.shape[0] >= 2:
            target_position, target_dist, target_angle = self.midpoint_polar(cluster_centers_f[0], cluster_centers_f[1])

        # Plotting
        # self.plot_scan(ranges, angles)        
        self.plot_clusters(ranges_m, angles_m, clusters, n_clusters)
        plt.polar(target_angle, target_dist, 'rx', markersize=10, label='Target') # Plotting the target of steering
        plt.polar(cluster_centers[:, 1], cluster_centers[:, 0], 'c^', markersize=10, label='Cone', alpha=0.5) # Plotting the (potential) cones


        # Axis and title names
        plt.title('Clusterized scan')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0., frameon=True)
        plt.tight_layout()
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.gca().yaxis.set_label_coords(-0.1,0.5) # Moving the y label to the left

        plt.pause(0.001)        # Pausing to allow the plot to be drawn and updated
        plt.clf()               # Clearing the plot             
        
        # Debugging
        if debug:
            # Scan
            print('Scan detected ', len(ranges_m), '(non-inf) ranges.')

            # Number of clusters found
            print('Scan clustered in ', n_clusters, ' clusters.')
            # Cluster cenetrs
            print('Cluster centers: ', self.find_cluster_centers(ranges_m, angles_m, clusters, n_clusters))

def main(args=None):
    rclpy.init(args=args)
    cone_visualizer = ConeVisualizer()
    rclpy.spin(cone_visualizer)
    cone_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
