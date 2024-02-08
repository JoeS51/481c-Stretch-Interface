import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

class MoveJoint(Node):
    def __init__(self):
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result