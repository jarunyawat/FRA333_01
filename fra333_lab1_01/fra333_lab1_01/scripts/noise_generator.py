#!/usr/bin/python3

# import all other neccesary libraries here
from rclpy.node import Node
from lab1_interfaces.srv import SetNoise
import sys
from std_msgs.msg import Float64
import numpy as np
import rclpy

class NoiseGenerator(Node):

    def __init__(self):
        super().__init__(f"noise_generator")
        # get the rate from argument or default
        if len(sys.argv)>1: 
            self.rate = float(sys.argv[1])
        else:
            self.rate = 5.0
        # add codes here
        self.set_noise_srv = self.create_service(SetNoise,"set_noise",self.set_noise_callback)
        self.pub_noise = self.create_publisher(Float64,"noise",10)
        self.period = self.create_timer(1.0/self.rate,self.timer_callback)
        # additional attributes
        self.mean = 0.0
        self.variance = 1.0
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()} with the default parameter. mean: {self.mean}, variance: {self.variance}')
    
    def set_noise_callback(self,request:SetNoise.Request,response:SetNoise.Response):
        # add codes here
        if request.variance.data < 0.0:
            self.get_logger().warning("variance cannot be negative")
        else:
            self.mean = request.mean.data
            self.variance = request.variance.data
        return response
    
    def timer_callback(self):
        # remove pass and add codes here
        msg = Float64()
        msg.data = np.random.normal(self.mean,np.sqrt(self.variance))
        self.pub_noise.publish(msg)

def main(args=None):
    # remove pass and add codes here
    rclpy.init(args=args)
    noiseGen = NoiseGenerator()
    rclpy.spin(noiseGen)
    noiseGen.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()