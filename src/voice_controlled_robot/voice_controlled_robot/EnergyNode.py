import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import time
import speech_recognition as sr
import pyaudio
import sounddevice




class EnergyNode(Node):
    def __init__(self):
        super().__init__('energy_node')
        self.subscription=self.create_subscription(
            Int32,
            'energy_status',
            self.energy_callback,
            10)
        self.get_logger().info("Energy Node Initialized")

    def energy_callback(self,msg):
        self.get_logger().info(f"Energy level: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    energynode=EnergyNode() 
    rclpy.spin(energynode)
    energynode.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()