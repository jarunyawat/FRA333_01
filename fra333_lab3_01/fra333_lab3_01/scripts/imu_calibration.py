#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from fra333_lab3_01_interface.action import ImuCalibrate
from sensor_msgs.msg import Imu
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float64
import time
import sys

class CalibrateActionServer(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.group_1 = MutuallyExclusiveCallbackGroup()
        self.group_2 = MutuallyExclusiveCallbackGroup()
        self.imu_sub = self.create_subscription(Imu,"/Imu_arduino",self.imu_callback,10,callback_group=self.group_1)
        self.action_server = ActionServer(self,
            ImuCalibrate,
            'imu_calibration',
            self.execute_callback,callback_group=self.group_2)
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])
        self.rate = self.create_rate(50)

    def imu_callback(self,data:Imu):
        self.angular_velocity = np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
        self.linear_acceleration = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
        # self.get_logger().info("imu callback")
        # self.get_logger().info(f"\nangular_velocity: {self.angular_velocity}\nlinear_acceleration: {self.linear_acceleration}\nrate: {1.0/(time.time()-start_time)}")
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        period = goal_handle.request.time.data
        angular_velocity_list = list()
        linear_acceleration_list = list()
        start_timer = time.time()
        while time.time() - start_timer < period:
            angular_velocity_list.append(self.angular_velocity)
            linear_acceleration_list.append(self.linear_acceleration)
            # self.get_logger().info(f"\nangular_velocity: {self.angular_velocity}\nlinear_acceleration: {self.linear_acceleration}")
            # self.get_logger().info("action callback")
            self.rate.sleep()
        angular_velocity_cov = np.cov(np.array(angular_velocity_list).T)
        linear_acceleration_cov = np.cov(np.array(linear_acceleration_list).T)
        angular_velocity_cov = angular_velocity_cov.ravel()
        linear_acceleration_cov = linear_acceleration_cov.ravel()
        self.get_logger().info(f"\nangular_velocity_cov: {angular_velocity_cov}\nlinear_acceleration_cov: {linear_acceleration_cov}")
        goal_handle.succeed()
        self.get_logger().info(f"finish calibration")
        result = ImuCalibrate.Result()
        for i in range(9):
            v_data = Float64()
            a_data = Float64()
            v_data.data = angular_velocity_cov[i]
            a_data.data = linear_acceleration_cov[i]
            result.angular_velocity_covariance[i] = v_data
            result.linear_acceleration_covariance[i] = a_data
        return result

def main(args=None):
    rclpy.init(args=args)
    execute = MultiThreadedExecutor(num_threads=2)
    action_node = CalibrateActionServer("calibration_backend")
    execute.add_node(action_node)
    try:
        while rclpy.ok():
            execute.spin_once()
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        action_node.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()