#!/usr/bin/env python3
import time
import json
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np


class ExecuteTrajectoryToy(Node):
    def __init__(self):
        unique_id = str(uuid.uuid4()).replace('-', '_')
        node_name = f'execute_trajectoryToy_{unique_id}'
        super().__init__(node_name)
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            exit(1)

        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.subscription
        self.get_logger().info('1')
        self.done = False

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def execute(self):
        rotate_point = JointTrajectoryPoint()
        with open('value_toy.json', 'r') as f:
            values = json.load(f)
            rotate_point.positions = [values['rotation']+.1]
        rotate_point.time_from_start = Duration(seconds=5.0).to_msg()

        rotate_goal = FollowJointTrajectory.Goal()
        rotate_goal.trajectory.joint_names = ['rotate_mobile_base']
        rotate_goal.trajectory.points = [rotate_point]
        self.trajectory_client.send_goal_async(rotate_goal)
        # camera_goal = FollowJointTrajectory.Goal()
        # camera_goal.trajectory.joint_names = ['joint_head_pan']
        # camera_goal.trajectory.points = [-rotate_point]
        # self.trajectory_client.send_goal_async(camera_goal)


        time.sleep(5)

        translate_point = JointTrajectoryPoint()
        with open('value_toy.json', 'r') as f:
            values = json.load(f)
            rotate_point.positions = [values['distance'] -0.5 ]
        translate_point.time_from_start = Duration(seconds=5.0).to_msg()

        translate_goal = FollowJointTrajectory.Goal()
        translate_goal.trajectory.joint_names = ['translate_mobile_base']
        translate_goal.trajectory.points = [rotate_point]
        self.trajectory_client.send_goal_async(translate_goal)



        time.sleep(5)
        rotate_point = JointTrajectoryPoint()
        
        rotate_point.positions = [3.625/2]
        rotate_point.time_from_start = Duration(seconds=5.0).to_msg()

        rotate_goal = FollowJointTrajectory.Goal()
        rotate_goal.trajectory.joint_names = ['rotate_mobile_base']
        rotate_goal.trajectory.points = [rotate_point]
        self.trajectory_client.send_goal_async(rotate_goal)    
        # camera_goal = FollowJointTrajectory.Goal()
        # camera_goal.trajectory.joint_names = ['joint_head_pan']
        # camera_goal.trajectory.points = [-rotate_point]
        # self.trajectory_client.send_goal_async(camera_goal)
        self.done = True
        print("done")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    trajectory_node = ExecuteTrajectoryToy()
    rclpy.spin_once(trajectory_node)
    trajectory_node.execute()
    rclpy.spin(trajectory_node)
    trajectory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()