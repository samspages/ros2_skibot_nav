import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from skibot_interfaces.msg import Pose
from geometry_msgs.msg import Wrench
import numpy as np

class SkibotNavNode(Node):

    def __init__(self):
        super().__init__('skibot_nav')
        self.target_x = 3.0 
        self.target_y = 4.0
        self.initial_theta = 0.0  # Matching the robot's initial orientation
        self.current_theta = self.initial_theta  # To keep track of the robot's current heading
        print("goal pose: x=" + str(self.target_x) + " y=" + str(self.target_y))
        self.state = "rotating"
        self.k_p_rotation = 0.1
        self.k_p_translation = 0.1

        self.subscription = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(Wrench, 'thrust', 10)

    def pose_callback(self, msg):
        self.current_theta = msg.theta  # Updating the current heading of the robot
        
        error_x = self.target_x - msg.x
        error_y = self.target_y - msg.y
        desired_theta = np.arctan2(error_y, error_x)
        error_theta = (desired_theta - self.current_theta + np.pi) % (2 * np.pi) - np.pi

        control_output = Wrench()
        distance_to_target = np.sqrt(error_x**2 + error_y**2)

        if self.state == "rotating":
            print("rotating from theta: " + str(self.current_theta) + " to goal theta: " + str(desired_theta))
            if abs(error_theta) < 0.1:  # Tolerance to be defined
                self.state = "translating"
            else:
                control_output.torque.z = self.k_p_rotation * error_theta

        elif self.state == "translating":
            print("moving at position x=" + str(msg.x) + " y= " + str(msg.y))
            if distance_to_target < 0.1:
                self.state = "stopped"
            else:
            # Check if the robot's heading deviates too much from the desired heading
                if abs(error_theta) > 0.2:  # Example threshold, adjust as needed
                    self.state = "rotating"
                else:
                    control_output.force.x = self.k_p_translation * error_x
                    control_output.force.y = self.k_p_translation * error_y
            
        elif self.state == "stopped":
            print("goal found")
            control_output.force.x = 0.0
            control_output.force.y = 0.0
            control_output.torque.z = 0.0

        self.publisher_.publish(control_output)

def main(args=None):
    rclpy.init(args=args)
    skibot_nav_node = SkibotNavNode()
    executor = MultiThreadedExecutor()
    executor.add_node(skibot_nav_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        skibot_nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
