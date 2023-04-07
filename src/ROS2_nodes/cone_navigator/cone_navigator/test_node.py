  
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path, Odometry
from math import atan2, sqrt

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.declare_parameter('kP', 0.1)
        self.declare_parameter('kI', 0.0)
        self.declare_parameter('kD', 0.0)
        self.declare_parameter('tolerance', 0.1)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_turn', 1.0)
        self.kP = self.get_parameter('kP').value
        self.kI = self.get_parameter('kI').value
        self.kD = self.get_parameter('kD').value
        self.tolerance = self.get_parameter('tolerance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.path = Path()
        self.odom = Odometry()
        self.desired_position = Point()
        self.current_position = Point()
        self.current_speed = 0.0
        self.current_turn = 0.0
        self.total_error = 0.0
        self.previous_error = 0.0
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription1 = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.subscription2 = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def path_callback(self, msg):
        self.path = msg
        if len(self.path.poses) > 0:
            self.desired_position = self.path.poses[0].pose.position

    def odom_callback(self, msg):
        self.odom = msg
        self.current_position = self.odom.pose.pose.position
        self.current_speed = self.odom.twist.twist.linear.x

    def calculate_error(self):
        error = atan2(self.desired_position.y - self.current_position.y,
                      self.desired_position.x - self.current_position.x)
        return error

    def calculate_total_error(self, error):
        self.total_error += error

    def calculate_derivative_error(self, error):
        derivative_error = error - self.previous_error
        self.previous_error = error
        return derivative_error

    def calculate_control_signal(self, error, derivative_error):
        proportional = self.kP * error
        integral = self.kI * self.total_error
        derivative = self.kD * derivative_error
        control_signal = proportional + integral + derivative
        return control_signal

    def calculate_turn(self, control_signal):
        turn = max(-self.max_turn, min(self.max_turn, control_signal))
        return turn

    def calculate_speed(self, error, control_signal):
        speed = max(0.0, min(self.max_speed, self.current_speed + control_signal))
        if abs(error) > 1.57:
            speed = -self.max_speed
        return speed

    def run(self):
        while rclpy.ok():
            if len(self.path.poses) > 0:
                error = self.calculate_error()
                self.calculate_total_error(error)
                derivative_error = self.calculate_derivative_error(error)
                control_signal = self.calculate_control_signal(error, derivative_error)
                turn = self.calculate_turn

                speed = self.calculate_speed(error, control_signal)
                twist = Twist()
                twist.linear.x = speed
                twist.angular.z = turn
                self.publisher.publish(twist)
                distance_to_goal = sqrt((self.current_position.x - self.desired_position.x) ** 2 +
                                        (self.current_position.y - self.desired_position.y) ** 2)
                if distance_to_goal < self.tolerance:
                    self.path.poses.pop(0)
                    self.total_error = 0.0
                    self.previous_error = 0.0
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
