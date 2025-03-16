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

def main(args=None):
    rclpy.init(args=args)
    speakernode=SpeakerNode()
    rclpy.spin(speakernode)
    speakernode.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()