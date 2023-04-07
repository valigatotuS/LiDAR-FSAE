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
        super().__init__('cone_navigator')

        # Tuneable node parameters
        self.angular_speed = 0.2 # rad/s (max is 2.84 rad/s)
        self.linear_speed = 0.15 # m/s (max is 0.22 m/s)
        self.cluster_th = 0.1 # m           # clustering threshold
        self.min_samples = 5                # minimum number of samples in a cluster
        self.hfov = 120 # degrees           # horizontal field of view of the lidar camera

        # Variables
        self.target_position = [0.0, 0.0]   # target point coordinates, this is the point the robot will drive to
        self.target_dist = 0.0 # m          # distance between the robot and the target point
        self.target_angle = 0.0 # rad       # angle between the robot and the target point
        self.vel_msg = Twist()              # velocity message (linear and angular velocity, 0 by default)
        self.moving = False                 # flag to indicate if the robot is moving
        self.obstacle = False               # flag to indicate if the robot is facing an obstacle

        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1000)

        # Subscribtions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile= qos_profile_sensor_data # 5Hz
        )
        
        # Stopping the robot at initialization
        self.vel_publisher.publish(self.vel_msg) 

        # Logging node initialization
        self.info('Cone navigator node initialized.')

        return
    
    # --- Logging functions --- #

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return
    
    # --- Mathemathical functions --- #

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
    
    def find_target(self, cluster_centers):
        # Filtering clusters out that are too close to the robot
        cluster_centers = cluster_centers[cluster_centers[:, 0] > 0.1] 

        if len(cluster_centers) < 2:
            self.warn('Not enough clusters found, %i found' % len(cluster_centers))
            return None, None, None
        
        # Sort the clusters by distance
        cluster_centers = cluster_centers[cluster_centers[:, 0].argsort()]
        
        # Calculating the position, distance and angle between the midpoint and the origin
        target_position, target_dist, target_angle = self.midpoint_polar(cluster_centers[0], cluster_centers[1])

        return target_position, target_dist, target_angle
    
    # --- Steering functions --- #

    def turn(self, vel_msg, angle = 0, angular_speed = 0.2):
        vel_msg.linear.x = 0.0
        vel_msg.angular.z =  angular_speed if angle > 0 else -angular_speed
        self.vel_publisher.publish(vel_msg)
        self.info("\tturning...")
        time.sleep(abs(angle/self.angular_speed))
        self.vel_msg.angular.z = 0.0
        self.vel_publisher.publish(self.vel_msg) # stop

    def move(self, vel_msg, length = 0.25):
        vel_msg.linear.x = self.linear_speed
        vel_msg.angular.z = 0.0
        self.vel_publisher.publish(vel_msg)
        self.info("\tmoving forward...")        
        time.sleep(abs(length / self.linear_speed)) # letting the robot move
        vel_msg.linear.x = 0.0
        self.vel_publisher.publish(vel_msg) # stop

    def drive2point(self, target_dist, target_angle):   # /!\ note /!\ works well at low speeds, less at high speeds
        # simple driving logic (turning, then moving forward)
        self.moving = True

        if (abs(target_angle) > np.deg2rad(4)):
            self.turn(self.vel_msg, target_angle, self.angular_speed)

        if (target_dist > 0.05) and not self.obstacle:
            self.move(self.vel_msg, target_dist + 0.1)

        self.vel_msg.linear.x = self.linear_speed
        self.vel_msg.angular.z =  self.angular_speed if target_angle > 0 else -self.angular_speed

        self.moving = False

    def drive2point_curve(self):    # /!\ note /!\ works bad at low speeds, less at high speeds
        # smooth driving logic
        self.moving = True

        self.debug("\tdriving to point...")

        while True:
            if (abs(self.target_angle) > np.deg2rad(3)):
                # turn and move forward at the same time
                self.vel_msg.linear.x = self.linear_speed*0.3
                self.vel_msg.angular.z = self.angular_speed if self.target_angle > 0 else -self.angular_speed
                self.vel_publisher.publish(self.vel_msg)
                time.sleep(abs(self.target_angle / self.angular_speed))
            elif (self.target_dist > 0.05):
                # move forward
                self.vel_msg.linear.x = self.linear_speed
                self.vel_msg.angular.z = 0.0
                self.vel_publisher.publish(self.vel_msg)
                time.sleep(abs(self.target_dist / self.linear_speed))
            else:
                # stop the robot
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0
                self.vel_publisher.publish(self.vel_msg)
                break

        self.moving = False

    # --- Callback functions --- #
    
    def scan_callback(self, msg): 
        # Getting the ranges and angles from the message
        ranges = np.array(msg.ranges) # meters
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment) # radians

        # # Obstacle avoidance
        # if self.moving or self.obstacle:
        #     # If the robot is moving, check if there is an obstacle in front of it
        #     min_dist = 0.10     # is the minimum distance to an obstacle
        #     angle_range = 80    # is the number of points on each side of the robot
        #     self.obstacle = False    
        #     for i in range(0, angle_range):
        #         if (ranges[i] < min_dist) or (ranges[-i] < min_dist):
        #             self.obstacle = True
        #             break
        #     if self.obstacle:
        #         # If there is an obstacle, stop the robot
        #         self.vel_msg.linear.x = 0.0
        #         self.vel_msg.angular.z = 0.0
        #         self.vel_publisher.publish(self.vel_msg)
        #         self.moving = False
        #         self.warn("\tobstacle detected, stopping...")
        #         return

        # Defining the field of view of the lidar
        fov = self.hfov // 2 # field of view on each side
        ranges = np.concatenate([ranges[0:fov], ranges[-fov:]])
        angles = np.concatenate([angles[0:fov], angles[-fov:]])

        # Masking the infinities & zeros
        mask = np.isfinite(ranges) & (ranges > 0.01)
        ranges_m = ranges[mask]
        angles_m = angles[mask]

        # If not enough points are found, return
        if len(ranges_m) < 10:
            self.warn('Not enough points found')
            return

        # Clusterizing the scan
        coord = np.array([ranges_m * np.cos(angles_m), ranges_m * np.sin(angles_m)]).T
        clusters, n_clusters = self.clusterize_scan(coord, cluster_th=self.cluster_th, min_samples=self.min_samples)

        # Calculating the center of each cluster 
        cluster_centers = self.find_cluster_centers(ranges_m, angles_m, clusters, n_clusters)

        # Finding the target
        self.target_position, self.target_dist, self.target_angle = self.find_target(cluster_centers)

        # If no target is found, return
        if (self.target_position is None):
            self.warn('No target found')
            return

        # if the robot is already moving, do not apply driving logic
        if (not self.moving) and (self.target_position is not None): 
            # step by step driving logic (turning, then moving forward)
            drive_thread = Thread(target=self.drive2point, args=(self.target_dist, self.target_angle))
            drive_thread.start()
            # # smooth driving logic
            # drive_thread = Thread(target=self.drive2point_curve, args=())
            # drive_thread.start()
        
        # Debugging
        target_msg = ('Target position: x:%.2f y:%.2f' % (self.target_position[0], self.target_position[1]) + 
                        'Target distance: %.2f m' % self.target_dist +
                        'Target angle: %.2f degree' % np.rad2deg(self.target_angle))
        self.debug(target_msg)


################################## Main ##################################

def main(args=None):
    rclpy.init(args=args)
    cone_navigator = ConeNavigator()
    input("Press Enter to start...") # Blocking call, waiting for user to press enter to start the node
    rclpy.spin(cone_navigator)
    cone_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
