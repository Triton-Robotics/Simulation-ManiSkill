#!/usr/bin/env python3

import sim_node.ros_bridge
import rclpy
from rclpy.executors import MultiThreadedExecutor

def main():
    print("running sim node")
    rclpy.init()

    node = None

    try:
        node = sim_node.ros_bridge.Sim_Node()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print("shutting down sim node")
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
