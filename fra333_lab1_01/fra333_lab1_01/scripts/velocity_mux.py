#!/usr/bin/python3

# import all other neccesary libraries
import rclpy
from std_msgs.msg import Float64
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityMux(Node):
    def __init__(self):
        super().__init__("velocity_multiplexer")
        # get the rate from argument or default
        if len(sys.argv)>1:
            self.rate = float(sys.argv[1])
        else:
            self.rate = 5.0
        # add codes here
        self.linear_noise_sub = self.create_subscription(Float64,"/linear/velocity",self.linear_vel_sub_callback,10)
        self.angular_noise_sub = self.create_subscription(Float64,"/angular/velocity",self.angular_vel_sub_callback,10)
        self.velocyty_pub = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.period = self.create_timer(1.0/self.rate,self.timer_callback)
        self.v = 0.0
        self.w = 0.0
        # additional attributes
        self.cmd_vel = Twist()
        self.get_logger().info(f'Starting {self.get_name()}')

    def linear_vel_sub_callback(self,msg:Float64):
        # remove pass and add codes here
        self.v = msg.data
    
    def angular_vel_sub_callback(self,msg:Float64):
        # remove pass and add codes here
        self.w = msg.data
    
    def timer_callback(self):
        # remove pass and add codes here
        self.cmd_vel.linear.x = self.v
        self.cmd_vel.angular.z = self.w
        self.velocyty_pub.publish(self.cmd_vel)

def main(args=None):
    # remove pass and add codes here
    rclpy.init(args=args)
    veloMux = VelocityMux()
    rclpy.spin(veloMux)
    veloMux.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()