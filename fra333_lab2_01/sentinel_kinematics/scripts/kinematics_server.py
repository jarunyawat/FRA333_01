#!/usr/bin/python3

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sentinel_kinematics_interfaces.srv import SetPose, SolveIK, GetPose
import numpy as np
from math import sin, cos, atan2, pi
from scipy.optimize import minimize, fsolve
from scipy.spatial.transform import Rotation

# from dummy_kinematics.dummy_module import dummy_function, dummy_var

class Kinematics_server(Node):
    def __init__(self):
        super().__init__('kinematics_server')
        self.period = 10
        self.timer = self.create_timer(1/self.period,self.joint_pub_callback)
        self.set_joint_srv = self.create_service(SetPose,"set_joints",self.set_joint_callback)
        self.solve_ik_srv = self.create_service(SolveIK,"solve_ik",self.inverse_kinematics)
        self.get_pose_srv = self.create_service(GetPose,"get_pose",self.pose_callback)
        self.joint_pub = self.create_publisher(JointState,"joint_states",10)
        self.joint_name = ["link_0_to_1", "link_1_to_2", "link_2_to_3"]
        self.joint_config = [0.0, 0.0, 0.0]
        self.joint_max = [3, 3, 2]
        self.joint_min = [-3, 0, -2]
        self.bound = [(self.joint_min[0],self.joint_max[0]), (self.joint_min[1],self.joint_max[1]), (self.joint_min[2],self.joint_max[2])]
        self.n = 3
        self.DH = np.array([[0, 0, 150, pi/2],
                            [0, pi/2, 130, 0],
                            [390, -pi, 130, 0]])
        self.roh = np.array([1, 1, 1])
        self.H_e_n = np.array([[1, 0, 0, 360],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
        self.pose = self.forward_kinematics(self.joint_config)
    
    def set_joint_callback(self,request,response):
        if request.jointstate.position[0]<=self.joint_max[0] and request.jointstate.position[0]>=self.joint_min[0] and request.jointstate.position[1]<=self.joint_max[1] and request.jointstate.position[1]>=self.joint_min[1] and request.jointstate.position[2]<=self.joint_max[2] and request.jointstate.position[2]>=self.joint_min[2]:
            self.joint_config[0] = request.jointstate.position[0]
            self.joint_config[1] = request.jointstate.position[1]
            self.joint_config[2] = request.jointstate.position[2]
            self.pose = self.forward_kinematics([request.jointstate.position[0],-request.jointstate.position[1],request.jointstate.position[2]])
        response = SetPose.Response()
        response.taskspace.x = self.pose[0, -1]
        response.taskspace.y = self.pose[1, -1]
        response.taskspace.z = self.pose[2, -1]
        return response
    
    def pose_callback(self,request,response):
        response = GetPose.Response()
        response.taskspace.x = self.pose[0, -1]
        response.taskspace.y = self.pose[1, -1]
        response.taskspace.z = self.pose[2, -1]
        r = Rotation.from_matrix(self.pose[0:3,0:3])
        self.get_logger().info(f"{r.as_euler('xyz')}")
        return response
    
    def _cost(self, q, q_0):
        dq = np.array(q)-np.array(q_0)
        return np.dot(dq,dq)
    
    def _get_pose(self, q):
        temp = self.forward_kinematics(q)
        return [temp[0, -1], temp[1, -1], temp[2, -1]]
    
    def forward_kinematics(self, q):
        H = np.eye(4)
        for i in range(self.n):
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
            H = np.linalg.multi_dot([H.copy(),Tx,Rx,Tz,Rz,Hj])
        H_e_0 = np.dot(H.copy(),self.H_e_n)
        return H_e_0
    
    def inverse_kinematics(self,request,response):
        x = request.taskspace.x
        y = request.taskspace.y
        z = request.taskspace.z - self.DH[0, 2]
        gamma = 1
        if request.elbowup.data:
            gamma = -1
        l1 = self.DH[2, 0]
        l2 = self.H_e_n[0, -1]
        response = SolveIK.Response()
        if (l1-l2)*(l1-l2) <= x**2+y**2 and (l1+l2)*(l1+l2) >= x**2+y**2:
            joint_temp = self.joint_config.copy()
            c2 = (z**2+x**2+y**2-l1**2-l2**2)/(2*l1*l2)
            s2 = gamma * np.sqrt(1-c2**2)
            joint_temp[2] = atan2(s2, c2)
            joint_temp[1] = atan2(z,np.sqrt(x**2+y**2)) - atan2(l2*s2, l1+l2*c2)
            joint_temp[0] = atan2(y,x) - pi/2
            if joint_temp[0]<=self.joint_max[0] and joint_temp[0]>=self.joint_min[0] and joint_temp[1]<=self.joint_max[1] and joint_temp[1]>=self.joint_min[1] and joint_temp[2]<=self.joint_max[2] and joint_temp[2]>=self.joint_min[2]:
                self.joint_config = joint_temp
                self.pose = self.forward_kinematics(self.joint_config)
                response.status.data = True
        return response

    def joint_pub_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_name
        msg.position = self.joint_config
        msg.velocity = [0.5, 0.5, 0.5]
        msg.effort = [30.0, 30.0, 30.0]
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Kinematics_server()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__=='__main__':
    main()
