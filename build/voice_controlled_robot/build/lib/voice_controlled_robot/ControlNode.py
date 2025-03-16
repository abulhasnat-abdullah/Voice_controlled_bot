import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import time
import speech_recognition as sr
import pyaudio
import sounddevice






class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.subscription=self.create_subscription(  
            String,
            'voice_cmd',
            self.command_callback,
            10)
        self.energy_publisher=self.create_publisher(Int32, 'energy_status', 10)
        self.publisher_=self.create_publisher(Twist, 'cmd_vel', 10)
        self.energy=100

        self.get_logger().info("Control_Node Initialized.")

    def command_callback(self, msg):
        command=msg.data
        self.get_logger().info(f"Received command: {command}")
        twist=Twist()

        if "forward" in command:
            duration=self.extract_duration(command)
            if self.energy>0:
                self.get_logger().info(f"Moving Forward for {duration}s")
                twist.linear.x=0.2
                self.publisher_.publish(twist)
                time.sleep(duration)
                twist.linear.x=0.0
                self.publisher_.publish(twist)
                self.energy-=duration*2
        elif "left" in command:
            if self.energy>0:
                self.get_logger().info("Turning left")
                twist.angular.z=0.5
                self.publisher_.publish(twist)
                time.sleep(1)
                twist.angular.z=0.0
                self.publisher_.publish(twist)
                self.energy-=5

        elif "right" in command:
            if self.energy>0:
                self.get_logger().info("Turning right")
                twist.angular.z=-0.5
                self.publisher_.publish(twist)
                time.sleep(1)
                twist.angular.z=0.0
                self.publisher_.publish(twist)
                self.energy-=5
        elif command=="heal" or command=="kaboom":
            self.get_logger().info("Healing energy!")
            self.energy=min(self.energy+20, 100)
        elif command=="stop":
            self.get_logger().info("Stopping!")
            twist.linear.x=0.0
            twist.angular.z=0.0
            self.publisher_.publish(twist)
        
        self.publish_energy()

    def extract_duration(self, command):
        words=command.split()
        for i,word in enumerate(words):
            if word.isdigit():
                return int(word)
        return 2
    
    def publish_energy(self):
        msg=Int32()
        msg.data=self.energy
        self.energy_publisher.publish(msg)
        self.get_logger().info(f"Current Energy : {self.energy}")



def main(args=None):
    rclpy.init(args=args)
    controlnode=ControlNode()
    rclpy.spin(controlnode)      
    controlnode.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()