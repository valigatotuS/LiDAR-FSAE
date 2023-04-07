
import os, select, sys
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np, time, math, rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter


class PublishingSubscriber(Node):

  def __init__(self):
    super().__init__('move')
    print('Moving tb3 started...' + '\n' + 50*'-')
    
    # parameters
    self.linear_speed = 0.22  # m/s (max=0.22)
    self.angular_speed = 0.2 # rad/s

    # Create publisher cmd_vel ------------------------------------------------------------------------------------------------------
    self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1000)

    #create timers ----------------------------------------------------------------------------------------------------------------------
    timer_period = 5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
  
  # Publiceer cmd_vel elke sec ------------------------------------------------------------------------------------------------------------------
  def timer_callback(self):
    self.go_forward(-0.65)
    self.square_path(1)
    #self.publisher_cmd_vel.publish(self.actual_cmd)

  def go_forward(self, length = 0.25):
    vel_msg = Twist()
    vel_msg.linear.x = self.linear_speed
    self.vel_publisher.publish(vel_msg)
    print("\tmoving forward...")         
    time.sleep(abs(length / self.linear_speed)) # letting the robot move

  def turn(self, angle = 90):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = self.angular_speed
    self.vel_publisher.publish(vel_msg)
    print("\tturning left...") 
    time.sleep(abs((angle*np.pi/180)/self.angular_speed)) # 90 degrees turn time


  def square_path(self, length = 0.25):
    for i in range(4):
      self.go_forward(length=length) 
      self.turn(angle=90)

def main(args=None):
  rclpy.init(args=args)
  tb3_explore = PublishingSubscriber()
  rclpy.spin(tb3_explore)
  tb3_explore.destroy_node()
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()

