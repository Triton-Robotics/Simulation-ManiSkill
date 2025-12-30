#!/usr/bin/env python3
import sim_node.ros_bridge
import rclpy


def main():
    rclpy.init()
    node = sim_node.ros_bridge.Sim_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    print("shutting down sim node")    
    node.destroy_node()
    rclpy.try_shutdown()
    
main()
