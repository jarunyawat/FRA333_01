#!/usr/bin/python3
import numpy as np
from numpy import sin, cos, arctan2, pi
import numpy as np
import rclpy
import sys
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray, UInt32, Float64
from std_srvs.srv import Empty

class Controller(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.velocity_publihser = self.create_publisher(Float64MultiArray,"/forward_velocity_controller/commands",10)
        self.joint_feedback_sub = self.create_subscription(DynamicJointState,"/dynamic_joint_states",self.joint_feedback_callback,10)
        self.enb_srv = self.create_service(Empty, "sentinel/enable",self.enb_callback)
        self.arrival_cli = self.create_client(Empty, 'sentinel/arrival')
        self.control_rate = self.create_rate(100)
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
        self.calibrate = False
        self.state = "SET_HOME"
        self.J = None
        self.joint_control = [0.0, 0.0, 0.0]
        self.imu_data = [0.0, 0.0, 0.0]
        self.ws_feedback = None
        self.isEnb = False
        self.ws_goal = None
        # controller parameter
        self.Kp = 0.1
        self.Ki = 0.2
        self.Kd = 0.05
        self.error = 0
        self.error_sum = 0
        self.last_error = 0
        self.error_diff = 0

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

    def ik(self, ws):
        x = ws[0]
        y = ws[1]
        z = ws[2]
        gamma = -1
        l1 = self.DH[2, 0]
        l2 = self.H_e_n[0, -1]
        if (l1-l2)*(l1-l2) <= x**2+y**2 and (l1+l2)*(l1+l2) >= x**2+y**2:
            joint_temp = [0.0,0.0,0.0]
            c2 = (z**2+x**2+y**2-l1**2-l2**2)/(2*l1*l2)
            s2 = gamma * np.sqrt(1-c2**2)
            joint_temp[2] = -arctan2(s2, c2)
            joint_temp[1] = arctan2(z,np.sqrt(x**2+y**2)) - arctan2(l2*s2, l1+l2*c2)
            joint_temp[0] = arctan2(y,x) - pi/2
            if joint_temp[0]<=self.joint_max[0] and joint_temp[0]>=self.joint_min[0] and joint_temp[1]<=self.joint_max[1] and joint_temp[1]>=self.joint_min[1] and joint_temp[2]<=self.joint_max[2] and joint_temp[2]>=self.joint_min[2]:
                return joint_temp
        return self.joint_config

    def endEffectorJacobian(self,q):
        Jw = list()
        Jv = list()
        [R, P] = self.fk(q)
        # self.get_logger().info(f"matrix: {R[0]}")
        # self.get_logger().info(f"position: {R[-1][:3,-1]}")
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

    def joint_feedback_callback(self, data:DynamicJointState):
        joint_pos = list()
        for i in data.interface_values:
            joint_pos.append(i.values[0])
        self.joint_config = joint_pos
        _, P = self.fk(self.joint_config)
        self.ws_feedback = P[-1]
        self.J = self.endEffectorJacobian(self.joint_config)
    
    def enb_callback(self,request,response):
        self.isEnb = True
        self.ws_goal = request.destination
        return response

    def send_request(self):
        req = Empty.Request()
        self.future = self.arrival_cli.call_async(req)
        return self.future.result()

    def timer_callback(self):
        velocity_pub_msg = Float64MultiArray()
        if self.state == "SET_HOME":
            joint_home = [0.0, pi/2, pi/2]
            self.velocity = [0.0, 2*(joint_home[1]-self.joint_config[1]), 3*(joint_home[2]-self.joint_config[2])]
            if abs(joint_home[1]-self.joint_config[1]) < 0.1 and abs(joint_home[2]-self.joint_config[2]) < 0.1:
                self.state = "FORWARD"
                self.get_logger().info("FORWARD")
                self.send_request()
        elif self.state == "FORWARD":
            if self.isEnb:
                q_ref = self.ik(self.ws_ref)
                qd_ref = np.linalg.inv(self.J[3:]) @ self.wsd_ref
                self.error = q_ref - self.ik(self.ws_feedback)
                self.error_sum += self.error
                self.velocity = qd_ref + self.Kp * (self.error) + self.Ki * (self.error_sum) + self.Kd * (self.error_diff)
                self.error_diff = self.error - self.last_error
                self.last_error = self.error
                if self.proximity(self.ws_feedback, self.ws_goal):
                    self.isEnb = False
                    self.send_request()
            else:
                self.velocity = [0.0, 0.0, 0.0]
        velocity_pub_msg.data = self.velocity
        self.velocity_publihser.publish(velocity_pub_msg)
            
    def proximity(self,q, q_check):
        if (np.abs(q-q_check) < 0.00001).all():
            return 1
        return 0
 
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    try:
        while rclpy.ok():
            rclpy.spin_once(controller)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        controller.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()