import rclpy, tf2_ros, time, os, cv2
from scipy.spatial.distance import cdist
from scipy.spatial import ConvexHull
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import numpy as np, matplotlib.pyplot as plt
from rclpy.qos import qos_profile_sensor_data
from sklearn.cluster import DBSCAN
from threading import Thread
from scipy.spatial import Delaunay

class ConeRacer(Node):

    def __init__(self):
        super().__init__('cone_racer')

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

        # Class Variables
        self.mode = 'idle'                  # mode of the robot (idle, recognition, race)
        self.yaw = 0.0 # rad                # robot orientation
        self.position = np.array([0.0, 0.0, 0.0])                       # robot actual position
        self.target_position = np.array([0.0, 0.0, 0.0])                # target point coordinates, this is the point the robot will drive to
        self.target_dist = 0.0 # m          # distance between the robot and the target point
        self.target_angle = 0.0 # rad       # angle between the robot and the target point
        self.cluster_centers = np.array([], dtype=np.float32).reshape(0,2) # cluster centers
        self.scan = np.array([], dtype=np.float32)                      # laser scan data
        self.trajectory = np.array([], dtype=np.float32).reshape(0,2)   # trajectory of the robot
        self.ppath = np.array([], dtype=np.float32).reshape(0,2)        # planned path of the robot
        self.trajectory_index = 0                                       # index of the trajectory point the robot is driving to
        self.lap = 0                        # number of laps completed
        self.map_data = None                # map data
        self.vel_msg = Twist()              # velocity message (linear and angular velocity, 0 by default)
        self.at_start = True                # flag to indicate if the robot is at the start position
        self.moving = False                 # flag to indicate if the robot is moving
        self.obstacle = False               # flag to indicate if the robot is facing an obstacle
        self.tf_buffer = tf2_ros.Buffer()   # buffer to store the transformations
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) # listener to get the transformations

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1000)

        # Subscribtions
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile= qos_profile_sensor_data)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_profile= qos_profile_sensor_data)

        # Timers
        self.position_timer = self.create_timer(0.2, self.position_cb) # get robot position every 0.1 seconds
        
        # Stopping the robot at initialization
        self.vel_pub.publish(self.vel_msg) 

        # Logging finished  node initialization
        self.info('Cone racer node initialized.')

        # Setting the node in recognition mode
        self.mode = 'recognition'

        return
    
    # --- Logging functions --- #

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

    def info(self, msg):
        self.get_logger().info('<~' + self.mode.upper() + '~> ' + msg)
        return

    def warn(self, msg):
        self.get_logger().warn('<~' + self.mode.upper() + '~> ' + msg)
        return

    def error(self, msg):
        self.get_logger().error('<~' + self.mode.upper() + '~> ' + msg)
        return
    
    # --- Getters and setters --- #
    
    def get_transform(self, parent_frame, child_frame):
        try:
            trans = self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.error(str(e))
            return None
   
    def get_position(self):
        trans = self.get_transform("map", "base_link")
        if trans is not None:
            robot_pos = trans.transform.translation
        else:
            print('')
            return [0.0, 0.0, 0.0]
        return [robot_pos.x, robot_pos.y, robot_pos.z]
    
    def get_origin(self):
        while(self.position is None):
            print("Waiting for robot origin position...")
            self.position = self.get_position()
            self.info('Waiting for robot origin position...')
            time.sleep(1)
        return self.position
    
    # --- Mathemathical functions --- #

    def clusterize_scan(self, coord, cluster_th, min_samples):
        """
        Returns the clusters (by label) and the number of clusters.
        """
        dbscan = DBSCAN(eps=cluster_th, min_samples=min_samples)    
        dbscan.fit(coord)                   # Fitting the model to the data
        labels = dbscan.labels_             # Extracting the labels and number of clusters
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        return labels, n_clusters

    def find_cluster_centers(self, ranges, angles, clusters, n_clusters):
        """
        Returns the center of each cluster in polar coordinates
        """
        X = np.array([ranges, angles]).T
        centers = []
        for i in range(n_clusters):
            center = np.mean(X[clusters == i], axis=0)
            centers.append(center)
        return np.array(centers)
    
    def midpoint_polar(self, p1, p2):
        """
        Returns the midpoint of two polar points in polar coordinates
        """
        # Converting polar coordinates to cartesian coordinates
        x1, y1 = p1[0] * np.cos(p1[1]), p1[0] * np.sin(p1[1])
        x2, y2 = p2[0] * np.cos(p2[1]), p2[0] * np.sin(p2[1])

        # Calculating the midpoint
        midpoint = np.array([(x1 + x2) / 2, (y1 + y2) / 2])

        # Calculating the distance and angle between the midpoint and the origin
        distance = np.sqrt(midpoint[0] ** 2 + midpoint[1] ** 2)
        angle = np.arctan2(midpoint[1], midpoint[0])

        return midpoint, distance, angle
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Returns the euler angles (rad) from a quaternion (x, y, z, w).
        """
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
    
    def find_target(self, cones_polar_position: np.ndarray):
        """
        Returns the position, distance and angle between the midpoint of the two closest cones and the origin
        """

        if len(cones_polar_position) < 2:
            self.warn('Not enough cones found, %i found' % len(cones_polar_position))
            return None, 0.0, 0.0
        
        # Filtering cones out that are too close to the robot
        cones_polar_position = cones_polar_position[np.where(cones_polar_position[:, 0] > 0.2)]

        if len(cones_polar_position) < 2:
            self.warn('Not enough cones found, %i found' % len(cones_polar_position))
            return None, 0.0, 0.0
        
        # Sort the clusters by distance
        cones_polar_position = cones_polar_position[cones_polar_position[:, 0].argsort()]
        
        # Calculating the position, distance and angle between the midpoint and the origin
        target_position, target_dist, target_angle = self.midpoint_polar(cones_polar_position[0], cones_polar_position[1])
        # Calculating target position based on the robot position and orientation (with respect to the map and using polar coordinates)
        # target_position = self.position[:2] + target_ang

        return target_position, target_dist, target_angle
    
    def find_cone_centroids(self, map_data, resolution=0.05, xorigin=0, yorigin=0, threshold=42, white=101):
        """his 
        Returns the centroids of the cones out of an occupancy grid map
        """
        map_data_th = cv2.threshold(map_data, threshold, white, cv2.THRESH_BINARY)[1]
        contours, _ = cv2.findContours(map_data_th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[1:]

        centroids = [(cv2.minEnclosingCircle(cnt)[0][0] * resolution + xorigin,
                  cv2.minEnclosingCircle(cnt)[0][1] * resolution + yorigin) for cnt in contours]

        return centroids

    def find_lanes(self, centroids):
        """ 
        Returns the indices of the points that form the left and right lane
        """
        if len(centroids) < 2:
            self.warn('Not enough cones found for computing left and right lane, %i found' % len(centroids))
            return None, None, None

        centroids = np.array(centroids) # converting centroids list to numpy array
        distances = cdist(centroids, centroids)
        np.fill_diagonal(distances, np.inf) # setting the diagonal elements to a very large value to avoid connecting a centroid to itself
        
        hull_all = ConvexHull(centroids) # first convex hull with all the centroids to find the outermost points
        hull_indices_all = hull_all.vertices.tolist() # getting the indices of the points that form the convex hull
        hull_indices_all.append(hull_indices_all[0]) # add the first point to the end to close the loop
        
        remaining_indices = np.setdiff1d(np.arange(len(centroids)), hull_indices_all[:-1]) # get the indices of the points not included in the first convex hull
        hull_remaining = ConvexHull(centroids[remaining_indices]) # second convex hull with the remaining centroids
        hull_indices_remaining = remaining_indices[hull_remaining.vertices].tolist() # getting the indices of the points that form the second convex hull
        hull_indices_remaining.append(hull_indices_remaining[0]) # adding the first point to the end to close the loop

        centerline = np.array([]).reshape(0, 2)
        if True:#self.mode == "racing":
            range_ = len(hull_indices_all) if len(hull_indices_all) <= len(hull_indices_remaining) else len(hull_indices_remaining)
            for i in range(range_ - 1):
                p1 = centroids[hull_indices_all[i]]
                p2 = centroids[hull_indices_remaining[i]]
                centerline = np.vstack((centerline, (p1 + p2) / 2))
            # close the loop with the first point of the centerline
            centerline = np.vstack((centerline, centerline[0]))

        # shift the centerline so that the robot is at the beginning
        centerline = np.roll(centerline, -np.argmin(np.linalg.norm(centerline - self.position[:2], axis=1)), axis=0)
            

        return hull_indices_all, hull_indices_remaining, centerline

    def plot_track(self, map_data, resolution=0.05, xorigin=0, yorigin=0, threshold=42, white=101):
        """
        Plots the track, robot's position, robot's orientation and robot's target
        """
        centroids = self.find_cone_centroids(map_data.copy(), resolution, xorigin, yorigin)
        ll_indices, rl_indices, centerline = self.find_lanes(centroids)

        #----------------------------------------------------
        # 6 nearest centroids to the robot
        x_cones = self.position[0] + self.cluster_centers[:, 0] * np.cos(self.cluster_centers[:, 1])
        y_cones = self.position[1] + self.cluster_centers[:, 0] * np.sin(self.cluster_centers[:, 1])
        # tri = Delaunay(np.array([x_cones, y_cones]).T)
        # plt.scatter(x_cones, y_cones, s=400, c='m', marker='*', alpha=0.4, label='cone')
        
        
        if len(centroids) >= 6:
            closest_centroids = np.array(centroids)[np.argsort(cdist([self.position[:2]], centroids))[0][:6]]
            
            tri = Delaunay(closest_centroids)
            points = closest_centroids.copy()


            points = points[np.argsort(np.dot(points - self.position[:2], np.array([np.cos(self.yaw + np.pi/2), np.sin(self.yaw + np.pi/2)])))]
            lpoints = points[:3]
            rpoints = points[3:]

            plt.scatter(points[:, 0], points[:, 1], s=400, c='y', marker='^' , label='left side cone')
            plt.scatter(lpoints[:, 0], lpoints[:, 1], s=400, c='b', marker='^', label='right side cone')
            
            # plot the mids that lie on a vertice with only one lpoint

            # # calculate the midpoints of each vertice
            # midpoints = []
            # for i in range(len(tri.simplices)):
            #     midpoints.append((points[tri.simplices[i][0]] + points[tri.simplices[i][1]]) / 2)
            #     midpoints.append((points[tri.simplices[i][1]] + points[tri.simplices[i][2]]) / 2)
            #     midpoints.append((points[tri.simplices[i][2]] + points[tri.simplices[i][0]]) / 2)
            # midpoints = np.array(midpoints)

            # calculate the midpoints that are between a lpoint and a rpoint and on a line of the delaunay triangulation
            midpoints = []
            for i in range(len(tri.simplices)):
                if (tri.simplices[i][0] < 3 and tri.simplices[i][1] > 3) or (tri.simplices[i][1] < 3 and tri.simplices[i][0] > 3):
                    midpoints.append((points[tri.simplices[i][0]] + points[tri.simplices[i][1]]) / 2)
                if (tri.simplices[i][1] < 3 and tri.simplices[i][2] > 3) or (tri.simplices[i][2] < 3 and tri.simplices[i][1] > 3):
                    midpoints.append((points[tri.simplices[i][1]] + points[tri.simplices[i][2]]) / 2)
                if (tri.simplices[i][2] < 3 and tri.simplices[i][0] > 3) or (tri.simplices[i][0] < 3 and tri.simplices[i][2] > 3):
                    midpoints.append((points[tri.simplices[i][2]] + points[tri.simplices[i][0]]) / 2)

            midpoints = np.array(midpoints)
            plt.scatter(midpoints[:, 0], midpoints[:, 1], s=40, c='g', marker='*', label='midpoints')

            

            plt.triplot(closest_centroids[:, 0], closest_centroids[:, 1], tri.simplices.copy(), c='r', alpha=0.4, label='scan', linewidth=4)
        

        #----------------------------------------------------
        if ll_indices is None or rl_indices is None:
            return
        
        # plotting the robot's last scan which are in polar coordinates, the indices are the angles and the values are the distances
        # if len(self.scan) > 0:
        #     scan = self.scan.copy()
        #     range = np.linspace(self.yaw, self.yaw + 2 * np.pi, len(scan))
        #     x_scan = self.position[0] + scan * np.cos(range)
        #     y_scan = self.position[1] + scan * np.sin(range)
        #     plt.scatter(x_scan, y_scan, s=5, c='r', marker='.', alpha=0.4, label='scan')

        
        # plotting the occupancy grid map
        plt.imshow(map_data, cmap='gray', origin='lower', extent=[xorigin, xorigin + map_data.shape[1] * resolution, yorigin, yorigin + map_data.shape[0] * resolution], label='map')
        # plotting the possible cones centroids
        plt.scatter(np.array(centroids)[:, 0], np.array(centroids)[:, 1], s=100, c='g', marker='^', alpha=0.4, label='cone')
        # plotting the left lane
        plt.plot(np.array(centroids)[ll_indices, 0], np.array(centroids)[ll_indices, 1], c='b', label='left track limit')
        # plotting the right lane
        plt.plot(np.array(centroids)[rl_indices, 0], np.array(centroids)[rl_indices, 1], c='y', label='right track limit')
        # plotting the centerline
        # plt.plot(np.array(centerline)[:, 0], np.array(centerline)[:, 1], 'g--', alpha=0.5, label='centerline')
        # plotting the centerline order
        # for i, txt in enumerate(range(len(centerline))):
        #     plt.annotate(txt, (centerline[i][0], centerline[i][1]))
        # plotting the robot position
        plt.plot(self.position[0], self.position[1] , 'ro', label='robot')
        # plotting the robot orientation (use yaw)
        plt.arrow(self.position[0], self.position[1], 0.2*np.cos(self.yaw), 0.2*np.sin(self.yaw), head_width=0.05, head_length=0.05, fc='r', ec='r')
        # plotting the trajectory
        plt.plot(self.trajectory[:,0], self.trajectory[:,1], 'r--', linewidth=0.5)
        # plotting the target position
        plt.plot(self.position[0] + np.cos(self.yaw+self.target_angle)*self.target_dist, self.position[1] + np.sin(self.yaw+self.target_angle)*self.target_dist, 'rx', alpha=0.2)
        plt.legend()
        # plot the legend next to the plot
        plt.grid(True, alpha=0.5)
        plt.title('Occupancy grid processing')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.legend(loc='upper right', fontsize='small')
        plt.axis('equal')
        plt.pause(0.001)        # Pausing to allow the plot to be drawn and updated
        plt.clf()               # Clearing the plot
            
    # --- Steering functions --- #

    def turn(self, vel_msg, angle = 0, angular_speed = 0.2):
        vel_msg.linear.x = 0.0
        vel_msg.angular.z =  angular_speed if angle > 0 else -angular_speed
        self.vel_pub.publish(vel_msg)
        self.info("\tturning...")
        time.sleep(abs(angle/angular_speed))
        self.vel_msg.angular.z = 0.0
        self.vel_pub.publish(self.vel_msg) # stop

    def move(self, vel_msg, length = 0.25):
        vel_msg.linear.x = self.get_parameter('linear_speed').value
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        self.info("\tmoving forward...")        
        time.sleep(abs(length / self.get_parameter('linear_speed').value)) # letting the robot move
        vel_msg.linear.x = 0.0
        self.vel_pub.publish(vel_msg) # stop

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
                self.vel_pub.publish(self.vel_msg)
                time.sleep(abs(self.target_angle / self.get_parameter('angular_speed').value))
            elif (self.target_dist > 0.05):
                # move forward
                self.vel_msg.linear.x = self.get_parameter('linear_speed').value
                self.vel_msg.angular.z = 0.0
                self.vel_pub.publish(self.vel_msg)
                time.sleep(abs(self.target_dist / self.get_parameter('linear_speed').value))
            else:
                # stop the robot
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0
                self.vel_pub.publish(self.vel_msg)
                break

        self.moving = False

    def follow_trajectory(self):
        
        trajectory = self.trajectory.copy()
        for waypoint in trajectory:
            # Plotting
            self.position = self.get_position() # [x,y,z]
            plt.plot(self.trajectory[:,0], self.trajectory[:,1], 'r--', linewidth=0.5)
            plt.plot(self.position[0] , self.position[1] , 'ro')
            plt.plot(waypoint[0], waypoint[1], 'rx')
            plt.axis('equal')
            plt.pause(0.001)        # Pausing to allow the plot to be drawn and updated
            plt.clf()               # Clearing the plot

            self.info("Following trajectory...")
            self.target_dist = np.sqrt((waypoint[0] - self.position[0])**2 + (waypoint[1] - self.position[1])**2)
            self.target_angle = np.arctan2(waypoint[1] - self.position[1], waypoint[0] - self.position[0])
            self.info("target_dist: " + str(self.target_dist) + "target_angle: " + str(np.rad2deg(self.target_angle)))
            self.drive2point(self.target_dist, self.target_angle)

    def controller(self):
        """
        Main controller function
        """
        if self.state == "idle":
            pass
        elif self.state == "recognition":
            pass
        elif self.state == "racing":
            pass

    # --- Checking states functions --- #

    def check_starting_position(self, start_radius = 0.1):
        """
        Checks if the robot is on the start position in order to count the laps
        """
        if self.position is None:
            pass
        elif not self.at_start and ((0 < abs(self.position[0]) and abs(self.position[0]) < start_radius)) and ((0 < abs(self.position[1]) and abs(self.position[1]) < start_radius)):
            self.at_start = True
            self.lap += 1

            # /!\ note /!\ move the code below to a new function
            input("Robot is on start position, %i lap(s) completed. Press enter to go race mod..." % self.lap)
            self.mode = "racing"
            self.info("Robot is now in racing mode")
            # self.follow_trajectory()
            #Thread(target=self.follow_trajectory, args=None)
        elif (abs(self.position[0]) > start_radius) and abs(self.position[1]) > start_radius:
            self.at_start = False        

    # --- Callback functions --- #

    def trajectory_cb(self, msg):
        print("trajectory_callback")
        trajectory = []

    def position_cb(self):
        trans = self.get_transform("map", "base_link")
        if trans is None:
            self.warn("Not able to fetch the position, no transform found between map and base_link")
        else:
            position = trans.transform.translation
            self.position = np.array([position.x, position.y, position.z])
            q = trans.transform.rotation # quaternion is a 4-tuple (x, y, z, w) which represents the rotation
            _, _, self.yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            self.info("Position fetched succesfully")

    def map_cb(self, msg):
        map_data = np.array(msg.data, dtype=np.uint8).reshape((msg.info.height, msg.info.width))
        map_data = np.flipud(map_data)  # Flip the image vertically
        map_data = 100 - map_data       # Invert the colors
        self.map_data = map_data

        # import pickle
        # with open('map_data2.pickle', 'wb') as f:
        #     pickle.dump(map_data, f)

        self.plot_track(map_data, xorigin=msg.info.origin.position.x, yorigin=msg.info.origin.position.y, resolution=msg.info.resolution)

        # Plotting

        # trans = self.get_transform("map", "base_link")
        # if trans is not None:
        #     robot_pos = trans.transform.translation
        #     self.position = [robot_pos.x, robot_pos.y, robot_pos.z]
        #     self.trajectory = np.append(self.trajectory, [[robot_pos.x, robot_pos.y]], axis=0)
        #     q = trans.transform.rotation # quaternion is a 4-tuple (x, y, z, w) which represents the rotation
        #     roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        #     speed = self.get_speed()

        #     # Plotting the robot's position on the map
        #     plt.plot(robot_pos.x, robot_pos.y, 'ro', markersize=2) 
        #     # Plotting the robot's orientation the map
        #     plt.arrow(robot_pos.x, robot_pos.y, speed*np.cos(yaw), speed*np.sin(yaw), head_width=0.05, head_length=0.1, fc='r', ec='r') # Robot's orientation on the map
        #     # Draw the robot's FOV
        #     fov = np.deg2rad(self.get_parameter('hfov').value)
        #     plt.plot([robot_pos.x, robot_pos.x + 0.6*np.cos(yaw - fov/2)], [robot_pos.y, robot_pos.y + 0.6*np.sin(yaw - fov/2)], 'm--', label='FOV')
        #     plt.plot([robot_pos.x, robot_pos.x + 0.6*np.cos(yaw + fov/2)], [robot_pos.y, robot_pos.y + 0.6*np.sin(yaw + fov/2)], 'm--')

        # # Plotting the trajectory
        # if len(self.trajectory) >= 2:
        #     plt.plot(self.trajectory[:,0], self.trajectory[:,1], 'r--', linewidth=0.5)

        # # Plotting the map and rescaling it to the correct size
        # plt.title('X: %i, Y: %i, Z: %i' % (self.position[0], self.position[1], self.position[2]))
        # plt.imshow(map_data, cmap='gray', extent=[msg.info.origin.position.x, 
        #                                           msg.info.origin.position.x + msg.info.width*msg.info.resolution, 
        #                                           msg.info.origin.position.y, 
        #                                           msg.info.origin.position.y + msg.info.height*msg.info.resolution])
                                                  
        
        # # plot origin
        # plt.plot(msg.info.origin.position.x, msg.info.origin.position.y, 'bo', markersize=2)
        

        # # print("msg.info.origin.position.x: ", msg.info.origin.position.x)
        # # print("msg.info.origin.position.y: ", msg.info.origin.position.y)
        # # print("msg.info.width: ", msg.info.width)
        # # print("msg.info.height: ", msg.info.height)
        # # print("msg.info.resolution: ", msg.info.resolution)

        # # plt.title('SLAM Map: Cartographer')
        # plt.title('X: %.4f, Y: %.4f, Z: %.4f' % (self.position[0], self.position[1], self.position[2]))
        # plt.axis('equal')               # Setting the aspect ratio to 1
        # plt.pause(0.001)        # Pausing to allow the plot to be drawn and updated
        # plt.clf()               # Clearing the plot
 

    def scan_cb(self, msg): 
        if self.mode == 'racing' and self.moving:
            return
        elif (self.mode == 'racing') and (not self.moving):
            # if too close to the waypoint, move to the next one
            while np.sqrt((self.trajectory[self.trajectory_index][0] - self.position[0])**2 + (self.trajectory[self.trajectory_index][1] - self.position[1])**2) < 0.20:
                self.trajectory_index += 1 
            
            self.trajectory_index %= len(self.trajectory)
            # cartesian coordinates
            waypoint = self.trajectory[self.trajectory_index] 
            # convert to polar coordinates
            plt.plot(waypoint[0], waypoint[1], 'gx', alpha=0.5)
            self.target_dist = np.sqrt((waypoint[0] - self.position[0])**2 + (waypoint[1] - self.position[1])**2)
            self.target_angle = np.arctan2(waypoint[1] - self.position[1], waypoint[0] - self.position[0])-self.yaw
            
            # start driving to the waypoint
            print('distance to waypoint: ', self.target_dist, 'angle to waypoint: ', np.rad2deg(self.target_angle))
            input('Press enter to continue')
            race_thread = Thread(target=self.drive2point, args=(self.target_dist, self.target_angle))
            race_thread.start()

            self.trajectory_index += 1
            return 

        # Getting the ranges and angles from the message
        ranges = np.array(msg.ranges) # meters
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment) # radians

        self.check_starting_position()

        if self.mode == "racing":
            return

        # Defining the field of view of the lidar
        fov = self.get_parameter('hfov').value // 2 # field of view on each side
        ranges_fov = np.concatenate([ranges[0:fov], ranges[-fov:]])
        angles_fov = np.concatenate([angles[0:fov], angles[-fov:]])

        # Masking the infinities & zeros
        mask = np.isfinite(ranges_fov) & (ranges_fov > 0.01)
        ranges_m = ranges_fov[mask]
        angles_m = angles_fov[mask]

        # Masking the infinities & zeros of the scan
        mask2 = np.isfinite(ranges) & (ranges > 0.01)
        ranges_m2 = ranges[mask2]
        self.scan = ranges_m2

        # If not enough points are found, return
        if len(ranges_m) < 10:
            self.warn('Not enough points found')
            return

        # Clusterizing the scan
        coord = np.array([ranges_m * np.cos(angles_m), ranges_m * np.sin(angles_m)]).T
        clusters, n_clusters = self.clusterize_scan(coord, cluster_th=self.get_parameter('cluster_th').value, min_samples=self.get_parameter('min_samples').value)

        # Calculating the center of each cluster 
        cluster_centers = self.find_cluster_centers(ranges_m, angles_m, clusters, n_clusters)
        self.cluster_centers = cluster_centers


        # Finding the target
        self.target_position, self.target_dist, self.target_angle = self.find_target(cluster_centers)

        # If no target is found, return
        if (self.target_position is None):
            self.warn('No target found')
            return

        # if the robot is already moving, do not apply driving logic
        if (not self.moving) and (self.target_position is not None): 
            # step by step driving logic (turning, then moving forward)
            self.trajectory = np.append(self.trajectory, [[self.position[0], self.position[1]]], axis=0)
            drive_thread = Thread(target=self.drive2point, args=(self.target_dist, self.target_angle))
            drive_thread.start()
        
        # Debugging
        target_msg = ('Target position: x:%.2f y:%.2f' % (self.target_position[0], self.target_position[1]) + 
                        'Target distance: %.2f m' % self.target_dist +
                        'Target angle: %.2f degree' % np.rad2deg(self.target_angle))
        self.debug(target_msg)


################################## Main ##################################

def main(args=None):
    rclpy.init(args=args)
    cone_racer = ConeRacer()
    input("Press Enter to start...") # Blocking call, waiting for user to press enter to start the node

    rclpy.spin(cone_racer)           # Starts the node
    
    cone_racer.destroy_node()   
    rclpy.shutdown()

if __name__ == '__main__':
    main()
