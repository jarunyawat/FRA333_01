#!/usr/bin/python3
import numpy as np
from numpy import sin, cos, arctan2, pi
import rclpy
import sys
from rclpy.node import Node
from rclpy.action import ActionServer
from sentinel_interface.action import TrajectoryGoal

class Generator(Node):

    def __init__(self):
        super().__init__('generator_node')
        self._action_server = ActionServer(self,TrajectoryGoal,'generate_path',self.execute_callback)
        self.joint_max = [3, 3, 2]
        self.joint_min = [-3, 0, -2]
        self.velocity = [0.0, 0.0, 0.0]
        self.DH = np.array([[0, 0, 0.150, pi/2],
                            [0, pi/2, 0.130, 0],
                            [0.390, -pi, 0.130, 0]])
        self.roh = np.array([1, 1, 1])
        self.H_e_n = np.array([[1, 0, 0, 0.360],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
        self.joint_config = [0.0,0.0,0.0]
        self.ws_feedback = None
        self.isEnb = False
        self.ws_start = None
        self.ws_goal = None
        #trajectoty parameter
    
    def execute_callback(self, goal_handle):
        #compute trajectory
        tf = 0.1
        ws_i = np.array([goal_handle.request.startpos.x, goal_handle.request.startpos.y, goal_handle.request.startpos.z])
        ws_f = np.array([goal_handle.request.goalpos.x, goal_handle.request.goalpos.y, goal_handle.request.goalpos.z])
        length = np.linalg.norm(ws_f-ws_i)
        v_max = goal_handle.request.v_max.data
        a_max = goal_handle.request.a_max.data
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
        goal_handle.succeed()
        result = TrajectoryGoal.Result()
        result.a.data = A
        result.b.data = B
        result.c.data = C
        result.timeperiod.data = tf
        return result
 
def main(args=None):
    rclpy.init(args=args)
    generator = Generator()
    try:
        while rclpy.ok():
            rclpy.spin_once(generator)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()