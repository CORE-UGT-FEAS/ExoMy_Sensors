#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from exomy_msgs.msg import MotorCommands
from std_msgs.msg import Float32MultiArray
import math

class MotorCommandsListener(Node):
    def __init__(self):
        # Initialize the class variables
        super().__init__('motor_commands_listener')
        self.mode = "stopped"  # Modes: 'driving', 'turning', 'stopped'
        self.angular_speed = 3 * 2 * math.pi / 11  # rad/s, approx (3 turns) * 2π / (11 sec)
        self.wheel_radius = 0.047  # m, radius of the wheel
        self.linear_velocity = 0.236  # m/s
        self.t = None  # Initialize time variable
        self.total_distance = 0  # Total distance traveled
        self.x = 0  # Initial X position (m)
        self.y = 0  # Initial Y position (m)
        self.heading = 0  # Initial robot heading (rad)

        # Subscribe to motor commands
        self.subscriber = self.create_subscription(
            MotorCommands,
            '/exomy/motor_commands_with_timestamp',
            self.callback,
            10)

        # Publisher for (x, y, heading)
        self.position_publisher = self.create_publisher(Float32MultiArray, '/exomy/position', 10)

        # Timer to update position
        self.timer = self.create_timer(0.1, self.integrate)  # 0.1s interval

    def callback(self, msg):
        timestamp = msg.header.stamp
        motor_speeds = msg.motor_speeds

        # Determine locomotion mode
        if motor_speeds[0] == motor_speeds[1] != 0:
            self.mode = "driving"
        elif motor_speeds[0] == -motor_speeds[1] and motor_speeds[0] != 0:
            self.mode = "turning"
            self.turn_direction = 1 if motor_speeds[0] > 0 else -1  # Determine turn direction
        else:
            self.mode = "stopped"

        if self.mode == "stopped" and self.t is not None:
            self.get_logger().info(f"Stopped at {timestamp.sec}.{timestamp.nanosec}s")
            self.t = None  # Reset time when stopping
        else:
            self.t = timestamp.sec * 1e9 + timestamp.nanosec  # Store time in nanoseconds

    def integrate(self):
        if self.mode != "stopped" and self.t is not None:
            t2 = self.get_clock().now().nanoseconds
            time_diff = (t2 - self.t) / 1e9  # Convert to seconds
            self.t = t2

            if self.mode == "driving":
                self.x += self.linear_velocity * math.cos(self.heading) * time_diff
                self.y += self.linear_velocity * math.sin(self.heading) * time_diff
                self.total_distance += self.linear_velocity * time_diff

            elif self.mode == "turning":
                self.heading += self.turn_direction * self.angular_speed * time_diff  # Apply direction to heading

            self.get_logger().info(f"Mode: {self.mode}, Position: ({self.x}, {self.y}), Heading: {self.heading} rad")
            self.get_logger().info(f"Total distance: {self.total_distance} meters")

            # Publish (x, y, heading)
            msg = Float32MultiArray()
            msg.data = [self.x, self.y, self.heading]
            self.position_publisher.publish(msg)

    def spin(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    listener = MotorCommandsListener()
    listener.spin()
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
