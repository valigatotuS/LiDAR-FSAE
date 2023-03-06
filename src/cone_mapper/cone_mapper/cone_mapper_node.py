import rclpy, tf2_ros, geometry_msgs.msg, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import numpy as np, matplotlib.pyplot as plt
from rclpy.qos import qos_profile_sensor_data
from sklearn.cluster import DBSCAN
from threading import Thread

class ConeMapper(Node):

    def __init__(self):
        super().__init__('cone_mapper')

        # ROS2 parameters
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('angular_speed', None),
                    ('linear_speed', None),
                    ('cluster_th', None),
                    ('min_samples', None),
                    ('hfov', None),
                ],
            )

        # Variables
        self.position = np.array([0, 0, 0]) # robot position (x, y, z)
        self.target_position = [0.0, 0.0]   # target point coordinates, this is the point the robot will drive to
        self.target_dist = 0.0 # m          # distance between the robot and the target point
        self.target_angle = 0.0 # rad       # angle between the robot and the target point
        self.vel_msg = Twist()              # velocity message (linear and angular velocity, 0 by default)
        self.moving = False                 # flag to indicate if the robot is moving
        self.obstacle = False               # flag to indicate if the robot is facing an obstacle
        self.tf_buffer = tf2_ros.Buffer()   # buffer to store the transformations
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) # listener to get the transformations

        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1000)

        # Subscribtions
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile= qos_profile_sensor_data)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile= qos_profile_sensor_data)
        # self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, qos_profile= qos_profile_sensor_data)
        # self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # subscriber = node.create_subscription(Path, '/trajectory', callback)
        
        # Stopping the robot at initialization
        self.vel_publisher.publish(self.vel_msg) 

        # Logging finished  node initialization
        self.info('Cone mapper node initialized.')

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
    
    # --- Getters and setters --- #

    def get_speed(self):
        return self.vel_msg.linear.x
    
    def get_transform(self, parent_frame, child_frame):
        try:
            trans = self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(str(e))
            return None
    
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
    
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
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
        time.sleep(abs(angle/angular_speed))
        self.vel_msg.angular.z = 0.0
        self.vel_publisher.publish(self.vel_msg) # stop

    def move(self, vel_msg, length = 0.25):
        vel_msg.linear.x = self.get_parameter('linear_speed').value
        vel_msg.angular.z = 0.0
        self.vel_publisher.publish(vel_msg)
        self.info("\tmoving forward...")        
        time.sleep(abs(length / self.get_parameter('linear_speed').value)) # letting the robot move
        vel_msg.linear.x = 0.0
        self.vel_publisher.publish(vel_msg) # stop

    def drive2point(self, target_dist, target_angle):   # /!\ note /!\ works well at low speeds, less at high speeds
        # simple driving logic (turning, then moving forward)
        self.moving = True
        angular_speed = self.get_parameter('angular_speed').value

        if (abs(target_angle) > np.deg2rad(4)):
            self.turn(self.vel_msg, target_angle, angular_speed)

        if (target_dist > 0.05) and not self.obstacle:
            self.move(self.vel_msg, target_dist + 0.1)

        self.vel_msg.linear.x = self.get_parameter('linear_speed').value
        self.vel_msg.angular.z =  angular_speed if target_angle > 0 else -angular_speed

        self.moving = False

    def drive2point_curve(self):    # /!\ note /!\ works bad at low speeds, less at high speeds
        # smooth driving logic
        self.moving = True

        self.debug("\tdriving to point...")

        while True:
            if (abs(self.target_angle) > np.deg2rad(3)):
                # turn and move forward at the same time
                self.vel_msg.linear.x = self.get_parameter('linear_speed').value*0.3
                self.vel_msg.angular.z = self.get_parameter('angular_speed').value if self.target_angle > 0 else -self.get_parameter('angular_speed').value
                self.vel_publisher.publish(self.vel_msg)
                time.sleep(abs(self.target_angle / self.get_parameter('angular_speed').value))
            elif (self.target_dist > 0.05):
                # move forward
                self.vel_msg.linear.x = self.get_parameter('linear_speed').value
                self.vel_msg.angular.z = 0.0
                self.vel_publisher.publish(self.vel_msg)
                time.sleep(abs(self.target_dist / self.get_parameter('linear_speed').value))
            else:
                # stop the robot
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0
                self.vel_publisher.publish(self.vel_msg)
                break

        self.moving = False

    # --- Callback functions --- #

    def map_callback(self, msg):
        map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        map_data = np.flipud(map_data)  # Flip the image vertically
        map_data = 100 - map_data       # Invert the colors

        # Plotting

        trans = self.get_transform("map", "base_link")
        if trans is not None:
            robot_pos = trans.transform.translation
            q = trans.transform.rotation # quaternion is a 4-tuple (x, y, z, w) which represents the rotation
            roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            speed = self.get_speed()

            # Plotting the robot's position on the map
            plt.plot(robot_pos.x, robot_pos.y, 'ro', markersize=2) 
            # Plotting the robot's orientation the map
            plt.arrow(robot_pos.x, robot_pos.y, speed*np.cos(yaw), speed*np.sin(yaw), head_width=0.05, head_length=0.1, fc='r', ec='r') # Robot's orientation on the map
            # Draw the robot's FOV
            fov = np.deg2rad(self.get_parameter('hfov').value)
            plt.plot([robot_pos.x, robot_pos.x + 0.6*np.cos(yaw - fov/2)], [robot_pos.y, robot_pos.y + 0.6*np.sin(yaw - fov/2)], 'm--', label='FOV')
            plt.plot([robot_pos.x, robot_pos.x + 0.6*np.cos(yaw + fov/2)], [robot_pos.y, robot_pos.y + 0.6*np.sin(yaw + fov/2)], 'm--')

        # Plotting the map and rescaling it to the correct size
        plt.imshow(map_data, cmap='gray', extent=[msg.info.origin.position.x, 
                                                  msg.info.origin.position.x + msg.info.width*msg.info.resolution, 
                                                  msg.info.origin.position.y, 
                                                  msg.info.origin.position.y + msg.info.height*msg.info.resolution])
        plt.title('SLAM Map: Cartographer')
        plt.axis('equal')               # Setting the aspect ratio to 1
        plt.pause(0.001)        # Pausing to allow the plot to be drawn and updated
        plt.clf()               # Clearing the plot
 

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
        fov = self.get_parameter('hfov').value // 2 # field of view on each side
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
        clusters, n_clusters = self.clusterize_scan(coord, cluster_th=self.get_parameter('cluster_th').value, min_samples=self.get_parameter('min_samples').value)

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
    cone_mapper = ConeMapper()
    
    input("Press Enter to start...") # Blocking call, waiting for user to press enter to start the node

    rclpy.spin(cone_mapper)
    cone_mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
