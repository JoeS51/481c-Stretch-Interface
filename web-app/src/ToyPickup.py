import json
import rclpy
from rclpy.node import Node
import uuid
import time

from calculate import FrameListener
from executeToyBox import ExecuteTrajectoryToyBox

from executeToy import ExecuteTrajectoryToy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, qos_profile_system_default
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import json

from geometry_msgs.msg import PoseStamped
from BasicNavigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import MarkerArray

class ToyPickup(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.curr_pos = None
        self.filename = "./locations.json"
        self.navigator = BasicNavigator()
        self.is_processing = False

        self.toy_pickup = self.create_subscription(MarkerArray, '/aruco/marker_array', self.detect_marker, 10)

          # callback to store the location
        
    def detect_marker(self, marker):
        if self.is_processing:
            return  # Skip processing if we're already handling a marker
            # #self.get_logger().info(marker)
        if (len(marker.markers) == 0):
            return

        print(marker.markers[0].id)
        print("fun")

        for marker in marker.markers:
            if marker.id == 136:
                self.is_processing = True
                node_toybox = FrameListener()
                node_toybox.obj = 'toy_box'
                print(node_toybox.obj)
                #while node_toybox.done != True:
                rclpy.spin_once(node_toybox)
                node_toybox.on_timer
                try:
                    trajectory_node = ExecuteTrajectoryToyBox()
                    trajectory_node.execute()
                    rclpy.spin_once(trajectory_node)
                except RuntimeError as e:
                    print(e)
                finally:
                    print("inFanally toybox")
                    trajectory_node.destroy_node()
                    node_toybox.destroy_node()
                    time.sleep(5)
                    self.is_processing = False 
                break
            elif marker.id == 135:
                self.is_processing = True
                node_toy = FrameListener()
                node_toy.obj = 'toy'
                print(node_toy.obj)
                #while node_toy.done != True:
                rclpy.spin_once(node_toy)
                node_toy.on_timer
                try:
                    trajectory_node = ExecuteTrajectoryToy()
                    trajectory_node.execute()
                    rclpy.spin_once(trajectory_node)
                except RuntimeError as e:
                    print(e)
                finally:
                    print("inFanally toy")
                    node_toy.destroy_node()
                    trajectory_node.destroy_node()
                    time.sleep(5)
                    self.is_processing = False
                break

        # if(marker.markers[0].id != 135):
        #     print("wrong marker")
        #     return
        # else:
        #     print("ALIGN TO ARUCO")
        #     #AlignToAruco() 
        #     ExecuteTrajectory()

def main(args=None):
    # rclpy.init(args=args)
    
    # pickup = ToyPickup()
    # try:
    #     rclpy.spin(pickup)
    # except KeyboardInterrupt:
    #     pass
    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # pickup.destroy_node()
    # rclpy.shutdown()
    rclpy.init(args=args)
    unique_id = str(uuid.uuid4()).replace('-', '_')
    toy_pickup_node = ToyPickup(node_name=f'toy_pickup_node_{unique_id}')
    executor = rclpy.executors.SingleThreadedExecutor()

    executor.add_node(toy_pickup_node)

    try:
        # Use executor.spin() to keep everything running and responsive.
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        toy_pickup_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    