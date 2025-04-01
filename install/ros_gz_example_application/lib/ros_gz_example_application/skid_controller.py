#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
# If using Gazebo Ignition, you may need this instead
# from ros_gz_interfaces.msg import Float32 as IgnFloat

class SkidSteeringController(Node):
    def __init__(self):
        super().__init__('skid_steering_controller')
        
        # Create a subscription to the twist commands
        self.twist_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Topic from teleop_twist_keyboard
            self.twist_callback,
            10)
        
        # Create publishers for each wheel
        self.fr_pub = self.create_publisher(Float64, '/frwheel_to_leg_r/velocity_controller/command', 10)
        self.fl_pub = self.create_publisher(Float64, '/frwheel_to_leg_l/velocity_controller/command', 10)
        self.br_pub = self.create_publisher(Float64, '/bkwheel_to_leg_r/velocity_controller/command', 10)
        self.bl_pub = self.create_publisher(Float64, '/bkwheel_to_leg_l/velocity_controller/command', 10)
        
        # Robot parameters
        self.wheel_radius = 0.079  # From your URDF
        self.wheel_separation = 0.29  # From your URDF (distance between left and right wheels)
        
        self.get_logger().info('Skid Steering Controller has been started')
        
    def twist_callback(self, msg):

        lin_mult = 3
        ang_mult = 4
        # Extract linear and angular velocities from the Twist message
        linear_x = msg.linear.x * lin_mult
        angular_z = msg.angular.z * ang_mult
        
        # Calculate wheel velocities for skid steering
        left_velocity = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_velocity = linear_x + (angular_z * self.wheel_separation / 2.0)
        
        # Convert linear wheel velocity to angular velocity (radians/second)
        left_angular_velocity = left_velocity / self.wheel_radius
        right_angular_velocity = right_velocity / self.wheel_radius
        
        # Create messages
        fr_msg = Float64()
        fl_msg = Float64()
        br_msg = Float64()
        bl_msg = Float64()
        
        # Set velocities
        fr_msg.data = right_angular_velocity
        fl_msg.data = left_angular_velocity
        br_msg.data = right_angular_velocity
        bl_msg.data = left_angular_velocity
        
        # Publish the messages
        self.fr_pub.publish(fr_msg)
        self.fl_pub.publish(fl_msg)
        self.br_pub.publish(br_msg)
        self.bl_pub.publish(bl_msg)
        
        # Log the velocities
        self.get_logger().debug(f'Linear: {linear_x}, Angular: {angular_z}')
        self.get_logger().debug(f'Wheel velocities (rad/s) - FR: {fr_msg.data}, FL: {fl_msg.data}, BR: {br_msg.data}, BL: {bl_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    controller = SkidSteeringController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()