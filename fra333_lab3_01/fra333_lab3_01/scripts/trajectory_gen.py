#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros, sin, cos, pi
import numpy as np
import rclpy
import sys, os, yaml
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray, UInt32, Float64
from std_srvs.srv import Empty
from rclpy.action import ActionClient
from fra333_lab3_01_interface.action import ImuCalibrate
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import time

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/forward_velocity_controller/commands"
        self.velocity_publihser = self.create_publisher(Float64MultiArray,publish_topic,10)
        self.joint_feedback_sub = self.create_subscription(DynamicJointState,"/dynamic_joint_states",self.joint_feedback_callback,10)
        self.imu_sub = self.create_subscription(Imu,"/Imu_arduino",self.imu_callback,10)
        self.imu_filter_sub = self.create_subscription(Imu,"/imu/data",self.imu_filter_callback,10)
        self.imu_with_cov_pub = self.create_publisher(Imu,"/imu/data_raw",10)
        self._action_client = ActionClient(self, ImuCalibrate, 'imu_calibration')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['link_0_to_1','link_1_to_2','link_2_to_3']
        self.velocity = [0.0, 0.0, 0.0]
        self.is_change_point = False
        self.i = 0
        self.DH = np.array([[0, 0, 0.150, pi/2],
                            [0, pi/2, 0.130, 0],
                            [0.390, -pi, 0.130, 0]])
        self.roh = np.array([1, 1, 1])
        self.H_e_n = np.array([[1, 0, 0, 0.360],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
        self.joint_config = [0.0,0.0,0.0]
        self.send_goal(duration=5)
        self.calibrate = False
        self.state = "SET_HOME"
        self.J = None
        self.joint_control = [0.0, 0.0, 0.0]
        self.imu_data = [0.0, 0.0, 0.0]

    def fk(self,q):
        H = np.eye(4)
        R = list()
        P = list()
        for i in range(len(q)):
            Tx = np.eye(4)
            Tx[0, -1] = self.DH[i, 0]
            Rx = np.array([[1, 0, 0, 0],
                            [0, cos(self.DH[i, 1]), -sin(self.DH[i, 1]), 0],
                            [0, sin(self.DH[i, 1]), cos(self.DH[i, 1]), 0],
                            [0, 0, 0, 1]])
            Tz = np.eye(4)
            Tz[2, -1] = self.DH[i, 2]
            Rz = np.array([[cos(self.DH[i, 3]), -sin(self.DH[i, 3]), 0, 0],
                            [sin(self.DH[i, 3]), cos(self.DH[i, 3]), 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
            if self.roh[i] == 1:
                Hj = np.array([[cos(q[i]), -sin(q[i]), 0, 0],
                            [sin(q[i]), cos(q[i]), 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
            else:
                Hj = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, q[i]],
                            [0, 0, 0, 1]])
            rot = np.linalg.multi_dot([Tx,Rx,Tz,Rz,Hj])
            H = np.dot(H.copy(),rot)
            R.append(H.copy())
            P.append(H[0:3,-1])
        H_e_0 = np.dot(H.copy(),self.H_e_n)
        R.append(H_e_0)
        P.append(H_e_0[0:3,-1])
        return [R, P]

    def endEffectorJacobian(self,q):
        '''
            q : format list 1x3 [[i_11, i_12, i_13]]
            q unit: rad
            type something here

            return format list 6x3
            [ [i_11, i_12, i_13],
            [i_21, i_22, i_23],
            [i_31, i_32, i_33],
            [i_41, i_42, i_43],
            [i_51, i_52, i_53],
            [i_61, i_62, i_63] ]
        '''
        Jw = list()
        Jv = list()
        [R, P] = self.fk(q)
        # self.get_logger().info(f"matrix: {R[0]}")
        self.get_logger().info(f"position: {R[-1][:3,-1]}")
        p_e = P[-1]
        # self.get_logger().info(f"{p_e}")
        for j in range(len(q)):
            #Jw คือ unit vector ที่แสดงทิศทางของ joint ที่จะหมุนเทียบกับ global frame โดยที่ทุก joint เป็น revolute ่joint
            #ดังนั้นทิศทางหมุนบน local frame คือ [0,0,1] และต้องใช้ rotation matrix ในการแปลงให้ไปอยู่ใน global frame
            Jw.append(R[j][:3,:3] @ np.array([0,0,1]).T)
            #Jv คือทิศทางความเร็วเชิงเส้นของจุด origin บน global frame เมื่อเทียบจากการหมุนของจุดบน joint ต่างๆ
            Jv.append(np.cross(R[j][:3,:3] @ np.array([0,0,1]).T, (p_e-P[j])))
        Je = np.vstack((np.array(Jw).T,np.array(Jv).T))
        return Je
    
    def checkSingularity(self,q):
        '''
            q : format list 1x3 [[i_11, i_12, i_13]]
            q unit: rad
            type something here
            
            return format bool
        '''
        Je = self.endEffectorJacobianHW2(q)
        #เนื่องจากหุ่นยนต์มี 3 joint จึงเลือกใช้ jacobian 3 ล่าง ในการทำ j_star เพื่อนำไปหา determinant
        J_star = np.array(Je[3:,:])
        #เมื่อค่า absolute ของ determinant ของ J_star มีค่าเข้าใกล้ 0 จะทำให้หุ่นยนต์เกิสภาวะ singularity
        if np.abs(np.linalg.det(J_star)) <= 0.001:
            return True
        return False

    def joint_feedback_callback(self, data:DynamicJointState):
        joint_pos = list()
        for i in data.interface_values:
            joint_pos.append(i.values[0])
        self.joint_config = joint_pos
        # self.get_logger().info(f"{joint_pos}")
        self.J = self.endEffectorJacobian(self.joint_config)


    def timer_callback(self):
        if self.state == "SET_HOME":
            joint_home = [0.0, pi/3, pi/2]
            self.velocity = [0.0, 2*(joint_home[1]-self.joint_config[1]), 3*(joint_home[2]-self.joint_config[2])]
            if abs(joint_home[1]-self.joint_config[1]) < 0.1 and abs(joint_home[2]-self.joint_config[2]) < 0.1:
                self.state = "HOME"
                self.get_logger().info("HOME")
                self.counter_timer = time.time()
        elif self.state == "HOME":
            self.velocity = [0.0, 0.0, 0.0]
            if time.time() - self.counter_timer > 3:
                self.state = "FORWARD"
                self.counter_timer = time.time()
        elif self.state == "FORWARD":
            linear_velocity = np.array([0.0, 0.0, 0.0])
            if self.imu_data[0] >= 4:
                linear_velocity[0] = 0.2
            elif self.imu_data[0] < 4 and self.imu_data[0] >= -4:
                linear_velocity[0] = 0.0
            else:
                linear_velocity[0] = -0.2
            if self.imu_data[1] >= 4:
                linear_velocity[1] = 0.2
            elif self.imu_data[1] < 4 and self.imu_data[1] >= -4:
                linear_velocity[1] = 0.0
            else:
                linear_velocity[1] = -0.2
            joint_velocity = np.linalg.inv(self.J[3:,:]) @ linear_velocity
            self.velocity = [joint_velocity[0], joint_velocity[1], joint_velocity[2]]
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.velocity
        if self.calibrate:
            self.velocity_publihser.publish(velocity_pub_msg)
    
    def imu_callback(self,data:Imu):
        if self.calibrate:
            for i in range(9):
                v_data = self.angular_velocity_covariance[i].data
                a_data = self.linear_acceleration_covariance[i].data
                data.angular_velocity_covariance[i] = v_data
                data.linear_acceleration_covariance[i] = a_data
            self.imu_with_cov_pub.publish(data)
    
    def imu_filter_callback(self,data:Imu):
        self.imu_data = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
        # r = R.from_quat([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        # rpy = r.as_euler('zxy', degrees=False)
        # self.get_logger().info(f"rpy: {rpy}")
        # self.imu_data = [rpy[0], rpy[1], rpy[2]]
    
    def send_goal(self, duration:int):
        goal_msg = ImuCalibrate.Goal()
        time_calibrate = UInt32()
        time_calibrate.data = duration
        goal_msg.time = time_calibrate

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.angular_velocity_covariance = result.angular_velocity_covariance
        self.linear_acceleration_covariance = result.linear_acceleration_covariance
        self.get_logger().info(f"angular_velocity_covariance Result: {result.angular_velocity_covariance}")
        self.get_logger().info(f"linear_acceleration_covariance Result: {result.linear_acceleration_covariance}")
        self.calibrate = True
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(joint_trajectory_object)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        joint_trajectory_object.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()