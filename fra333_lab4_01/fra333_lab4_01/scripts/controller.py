#!/usr/bin/python3
import numpy as np
from numpy import sin, cos, arctan2, pi
import rclpy
import sys
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray
from fra333_lab4_01_interface.srv import Destination
from std_srvs.srv import Empty
import time

class Controller(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.velocity_publihser = self.create_publisher(Float64MultiArray,"/forward_velocity_controller/commands",10)
        self.joint_feedback_sub = self.create_subscription(DynamicJointState,"/dynamic_joint_states",self.joint_feedback_callback,10)
        self.enb_srv = self.create_service(Destination, "sentinel/enable",self.enb_callback)
        self.arrival_cli = self.create_client(Empty, 'sentinel/arrival')
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.joints = ['link_0_to_1','link_1_to_2','link_2_to_3']
        self.joint_max = [3, 3, 2]
        self.joint_min = [-3, 0, -2]
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
        self.ws_start = None
        self.ws_goal = None
        # controller parameter
        self.Kp = 1
        self.Ki = 0.05
        self.Kd = 10
        self.error = 0
        self.error_sum = 0
        self.last_error = 0
        self.error_diff = 0
        #trajectoty parameter
        self.A = 0
        self.B = 0
        self.C = 0
        self.timeStamp = time.time()
        self.timePeriod = 0

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
                return np.array(joint_temp).copy()
        return np.array(self.joint_config).copy()

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
    
    def checkSingularity(self,q):
        Je = self.endEffectorJacobian(q)
        #เนื่องจากหุ่นยนต์มี 3 joint จึงเลือกใช้ jacobian 3 ล่าง ในการทำ j_star เพื่อนำไปหา determinant
        J_star = np.array(Je[3:])
        #เมื่อค่า absolute ของ determinant ของ J_star มีค่าเข้าใกล้ 0 จะทำให้หุ่นยนต์เกิสภาวะ singularity
        if np.abs(np.linalg.det(J_star)) <= 0.001:
            return True
        return False

    def joint_feedback_callback(self, data:DynamicJointState):
        joint_pos = list()
        for i in data.interface_values:
            joint_pos.append(i.values[0])
        self.joint_config = joint_pos
        _, P = self.fk(self.joint_config)
        self.ws_feedback = P[-1]
        self.J = self.endEffectorJacobian(self.joint_config)
    
    def enb_callback(self,request,response):
        #compute trajectory
        tf = 0.1
        joint_i = self.joint_config
        [R, P] = self.fk(joint_i)
        ws_i = R[-1][:3,-1]
        ws_f = np.array([request.destination.x, request.destination.y, request.destination.z])
        a_max = 1
        v_max = 0.3
        length = np.linalg.norm(ws_f-ws_i)
        response = Destination.Response()
        while True:
            D = np.array([[5*tf**2, 4*tf, 3],
                    [20*tf**2, 12*tf, 6],
                    [tf**5, tf**4, tf**3]])
            Da = np.array([[0, 4*tf, 3],
                            [0, 12*tf, 6],
                            [length, tf**4, tf**3]])
            Db = np.array([[5*tf**2, 0, 3],
                            [20*tf**2, 0, 6],
                            [tf**5, length, tf**3]])
            Dc = np.array([[5*tf**2, 4*tf, 0],
                            [20*tf**2, 12*tf, 0],
                            [tf**5, tf**4, length]])

            A = np.linalg.det(Da)/np.linalg.det(D)
            B = np.linalg.det(Db)/np.linalg.det(D)
            C = np.linalg.det(Dc)/np.linalg.det(D)
            t_half = tf/2
            t_quater = tf/4
            if np.abs(5*A*t_half**4 + 4*B*t_half**3 + 3*C*t_half**2)<=v_max and np.abs(20*A*t_quater**3 + 12*B*t_quater**2 + 6*C*t_quater)<=a_max:
                break
            tf = 1.1*tf
        for t in np.arange(0,tf+self.dt,self.dt):
            # lenght_percent = (A*t**5 + B*t**4 + C*t**3)/length
            # pos_3d = (ws_f-ws_i)*lenght_percent + ws_i
            wsd_t = 5*A*t**4 + 4*B*t**3 + 3*C*t**2
            velo_3d = (ws_f-ws_i)/length * wsd_t
            J = self.endEffectorJacobian(joint_i)
            if self.checkSingularity(joint_i):
                response.success.data = False
                return response
            joint_qd = np.linalg.inv(J[3:]) @ velo_3d
            joint_i = joint_i + (joint_qd * self.dt)
        self.isEnb = True
        self.A = A
        self.B = B
        self.C = C
        self.timePeriod = tf
        self.timeStamp = time.time()
        self.ws_start = ws_i
        self.ws_goal = ws_f
        response.success.data = True
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
                t = time.time() - self.timeStamp
                if t <= self.timePeriod:
                    lenght_percent = (self.A*t**5 + self.B*t**4 + self.C*t**3)/np.linalg.norm(self.ws_goal-self.ws_start)
                    ws_t = (self.ws_goal-self.ws_start)*lenght_percent + self.ws_start
                    wsd_t = 5*self.A*t**4 + 4*self.B*t**3 + 3*self.C*t**2
                    velo_3d = (self.ws_goal-self.ws_start)/np.linalg.norm(self.ws_goal-self.ws_start) * wsd_t
                else:
                    ws_t = self.ws_goal
                    velo_3d = np.array([0.0, 0.0, 0.0])
                q_ref = self.ik(ws_t)
                qd_ref = np.linalg.inv(self.J[3:]) @ velo_3d
                self.error = q_ref - self.ik(self.ws_feedback)
                self.error_sum += self.error
                pid = qd_ref + self.Kp * (self.error) + self.Ki * (self.error_sum) + self.Kd * (self.error_diff)
                self.velocity = [pid[0], pid[1], pid[2]]
                self.error_diff = self.error - self.last_error
                self.last_error = self.error
                self.get_logger().info(f"{self.ws_feedback}")
                if self.proximity(self.ws_feedback, self.ws_goal):
                    self.isEnb = False
                    self.get_logger().info(f"destiantion arrival x:{self.ws_goal[0]} y:{self.ws_goal[1]} z:{self.ws_goal[2]}")
                    # self.send_request()
            else:
                self.velocity = [0.0, 0.0, 0.0]
        velocity_pub_msg.data = self.velocity
        self.velocity_publihser.publish(velocity_pub_msg)
            
    def proximity(self,q, q_check):
        if (np.abs(q-q_check) < 0.01).all():
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