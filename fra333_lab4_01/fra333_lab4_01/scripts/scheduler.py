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

class Scheduler(Node):

    def __init__(self):
        super().__init__('schedule_node')
        self.enb_cli = self.create_client(Empty, "sentinel/enable")
        self.arrival_srv = self.create_service(Empty, 'sentinel/enable', self.arrival_callback)

    def arrival_callback(self,request,response):
        self.send_request()
        return response
    
    def send_request(self):
        req = Empty.Request()
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