import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import time
import speech_recognition as sr
import pyaudio
import sounddevice

class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')
        self.publisher_=self.create_publisher(String, 'voice_cmd', 10) #topic - voice_cmd
        self.recognizer=sr.Recognizer()
        self.get_logger().info("Speaking Node Initialized. Say the commands...")
        self.listen_the_commands()

    def listen_the_commands(self):
        while rclpy.ok():
            with sr.Microphone() as mic:
                self.get_logger().info("Speak!")
                self.recognizer.adjust_for_ambient_noise(mic, duration=0.2)
                try:
                    audio = self.recognizer.listen(mic, timeout=5)  # Add timeout
                    command = self.recognizer.recognize_google(audio).lower()
                    self.get_logger().info(f"Recognized command: {command}")
                    msg = String()
                    msg.data = command
                    self.publisher_.publish(msg)

                    if command == "stop listening":
                        self.get_logger().info("Stopping listening for commands.")
                        break

                except sr.UnknownValueError:
                    self.get_logger().warn("Could not understand the command.")
                except sr.RequestError:
                    self.get_logger().warn("Speech Recognition Service Error")
                except Exception as e:
                    self.get_logger().error(f"An error occurred: {e}")

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
            self.get_logger().info("Turning left")
            twist.angular.z=0.5
            self.publisher_.publish(twist)
            time.sleep(1)
            twist.angular.z=0.0
            self.publisher_.publish(twist)
            self.energy-=5

        elif "right" in command:
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
    speakernode=SpeakerNode()
    controlnode=ControlNode()
    energynode=EnergyNode()
    try:
        rclpy.spin(speakernode)
        rclpy.spin(controlnode)
        rclpy.spin(energynode)
    except KeyboardInterrupt:
        pass
    speakernode.destroy_node()
    controlnode.destroy_node()
    energynode.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()


           

        
    



