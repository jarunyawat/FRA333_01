#!/usr/bin/python3
import numpy as np
from numpy import sin, cos, arctan2, pi
import numpy as np
import rclpy
import sys
from rclpy.node import Node
from std_srvs.srv import Empty
from fra333_lab4_01_interface.srv import Destination
from geometry_msgs.msg import Point

class Scheduler(Node):

    def __init__(self):
        super().__init__('schedule_node')
        self.enb_cli = self.create_client(Destination, "sentinel/enable")
        self.arrival_srv = self.create_service(Empty, 'sentinel/arrival', self.arrival_callback)
        self.last_node = None
        self.declare_parameters(namespace='', parameters=[
            ('x', []),
            ('y', []),
        ])
        # self.edge = [[(-0.4,0.4),(-0.4,0.6)], # F
        #             [(-0.4,0.6),(-0.25,0.6)], # F
        #             [(-0.4,0.5),(-0.25,0.5)], # F
        #             [(-0.15,0.6),(-0.15,0.4)], # I
        #             [(-0.2,0.6),(-0.1,0.6)], # I
        #             [(-0.2,0.4),(-0.1,0.4)], # I
        #             [(-0.05,0.6),(-0.05,0.4)], # B
        #             [(-0.05,0.6),(0.05,0.6)], # B
        #             [(0.05,0.6),(0.1,0.55)], # B
        #             [(0.1,0.55),(0.1,0.525)], # B
        #             [(0.1,0.525),(0.05,0.5)], # B
        #             [(0.05,0.5),(-0.05,0.5)], # B
        #             [(0.05,0.5),(0.1,0.475)], # B
        #             [(0.1,0.475),(0.1,0.45)], # B
        #             [(0.1,0.45),(0.05,0.4)], # B
        #             [(0.05,0.4),(-0.05,0.4)], # B
        #             [(0.2,0.6),(0.3,0.6)],
        #             [(0.3,0.6),(0.35,0.525)],
        #             [(0.35,0.525),(0.35,0.475)],
        #             [(0.35,0.475),(0.3,0.4)],
        #             [(0.3,0.4),(0.2,0.4)],
        #             [(0.2,0.4),(0.15,0.475)],
        #             [(0.15,0.475),(0.15,0.525)],
        #             [(0.15,0.525),(0.2,0.6)],] 
        x = self.get_parameter('x').get_parameter_value().double_array_value
        y = self.get_parameter('y').get_parameter_value().double_array_value
        self.edge = []
        self.get_logger().info(f"x: {x}")
        self.node_idx = 0
        self.edge_idx = 0
        self.state = "START"

    def arrival_callback(self,request,response):
        if self.edge_idx < len(self.edge):
            self.send_request()
        return response
    
    def send_request(self):
        req = Destination.Request()
        point = Point()
        if self.state == "START":
            self.node_idx = 0
            point.x = self.edge[self.edge_idx][self.node_idx][0]
            point.y = self.edge[self.edge_idx][self.node_idx][1]
            if self.last_node != None:
                if np.abs(point.x - self.last_node[0]) < 0.001 and np.abs(point.y - self.last_node[1]) < 0.001:
                    self.node_idx = 1
                    point.x = self.edge[self.edge_idx][self.node_idx][0]
                    point.y = self.edge[self.edge_idx][self.node_idx][1]
                    point.z = 0.1
                    self.last_node = [point.x, point.y]
                    req.write.data = True
                    self.edge_idx += 1
                    self.state = "START"
                else:
                    point.z = 0.2
                    self.state = "LIFT"
            else:
                point.z = 0.1
                self.state = "WRITE"
        elif self.state == "LIFT":
            point.x = self.edge[self.edge_idx][self.node_idx][0]
            point.y = self.edge[self.edge_idx][self.node_idx][1]
            point.z = 0.1
            self.state = "WRITE"
        elif self.state == "WRITE":
            self.node_idx = 1
            point.x = self.edge[self.edge_idx][self.node_idx][0]
            point.y = self.edge[self.edge_idx][self.node_idx][1]
            point.z = 0.1
            req.write.data = True
            self.last_node = [point.x, point.y]
            self.state = "START"
            self.edge_idx += 1
        self.get_logger().info(f"x:{point.x}, y:{point.y} z:{point.z}")
        req.destination = point
        self.future = self.enb_cli.call_async(req)
        return self.future.result()
 
def main(args=None):
    rclpy.init(args=args)
    scheduler = Scheduler()
    try:
        while rclpy.ok():
            rclpy.spin_once(scheduler)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        scheduler.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()