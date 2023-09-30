#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Point
import math
import time

class SkibotNavNode(Node):
    def __init__(self):
        super().__init__('skibot_nav_node')
        self.velocity_pub = self.create_publisher(Wrench, 'thrust', 10)
        self.goal_point = Point(x=-5.0, y=2.0)  # Set the goal position here
        self.rate = self.create_rate(10)  # Adjust the publishing rate as needed
        self.distance_tolerance = 0.1  # Tolerance for considering the goal reached

    def move_to_goal(self):
        while not self.goal_reached():
            # Calculate the direction vector towards the goal
            direction_x = self.goal_point.x
            direction_y = self.goal_point.y

            # Calculate the distance to the goal
            distance = math.sqrt(direction_x ** 2 + direction_y ** 2)

            # Normalize the direction vector
            direction_x /= distance
            direction_y /= distance

            # Calculate the tap force proportional to the distance
            tap_force = distance * 0.5  # Adjust the factor as needed

            wrench = Wrench()
            wrench.force.x = tap_force * direction_x  # Apply linear force
            wrench.force.y = tap_force * direction_y  # Apply linear force

            # Publish the wrench message to control the bot
            self.velocity_pub.publish(wrench)
            self.rate.sleep()

        # Stop the bot after reaching the goal
        self.stop_bot()

    def stop_bot(self):
        # Stop the bot by sending zero velocity
        zero_wrench = Wrench()
        self.velocity_pub.publish(zero_wrench)

    def goal_reached(self):
        # Check if the distance to the goal is within tolerance
        direction_x = self.goal_point.x
        direction_y = self.goal_point.y
        distance = math.sqrt(direction_x ** 2 + direction_y ** 2)
        return distance <= self.distance_tolerance

def main(args=None):
    rclpy.init(args=args)
    skibot_nav_node = SkibotNavNode()
    skibot_nav_node.move_to_goal()
    rclpy.spin(skibot_nav_node)
    skibot_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
