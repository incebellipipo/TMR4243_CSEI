#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tmr4243_interfaces.msg
import geometry_msgs.msg
import std_msgs.msg
import subprocess
import re
import csv
import os

class DataExtractor(Node):
    def __init__(self):
        super().__init__('data_extractor')
        self.subs = {}
       
        # Need to change these paths to the correct paths on your system
        self.bag_path = "/home/petter/bags/rosbag2_2024_04_08-14_19_10"
        self.csv_directory = "/home/petter/csvs/"

        self.csv_directory = self.csv_directory + os.path.basename(self.bag_path)
        if not os.path.exists(self.csv_directory):
            os.makedirs(self.csv_directory)
        
        
        subprocess.Popen(["ros2", "bag", "play", self.bag_path])
        topics_info = self.get_bag_info(self.bag_path)

        self.csv_files = {}
        self.csv_writers = {}
        self.latest_rosout_timestamp = None
        self.get_logger().info("Topics in bag file:")
        for topic, type_str in topics_info.items():
            self.get_logger().info('"%s"' % topic + ': ' + type_str)
            if topic == "/CSEI/observer/state":
                self.csv_files[topic] = open(os.path.join(self.csv_directory, topic.replace('/', '_') + ".csv"), 'w', newline='')
                self.csv_writers[topic] = csv.writer(self.csv_files[topic])
                self.csv_writers[topic].writerow(["eta.x", "eta.y", "eta.z", "nu.u", "nu.v", "nu.w", "bias.x", "bias.y", "bias.z"])
                self.create_subscription(tmr4243_interfaces.msg.Observer, topic, self.observer_callback, 10)
            elif topic == "/CSEI/control/u_cmd":
                self.csv_files[topic] = open(os.path.join(self.csv_directory, topic.replace('/', '_') + ".csv"), 'w', newline='')
                self.csv_writers[topic] = csv.writer(self.csv_files[topic])
                self.csv_writers[topic].writerow(["u_cmd.1", "u_cmd.2", "u_cmd.3", "u_cmd.4","u_cmd.5"])
                self.create_subscription(std_msgs.msg.Float32MultiArray, topic, self.control_callback, 10)
            elif topic == "/CSEI/state/tau":
                self.csv_files[topic] = open(os.path.join(self.csv_directory, topic.replace('/', '_') + ".csv"), 'w', newline='')
                self.csv_writers[topic] = csv.writer(self.csv_files[topic])
                self.csv_writers[topic].writerow(["tau.1", "tau.2", "tau.3"])
                self.create_subscription(std_msgs.msg.Float32MultiArray, topic, self.tau_callback, 10)
            elif topic == "/CSEI/state/eta":
                self.csv_files[topic] = open(os.path.join(self.csv_directory, topic.replace('/', '_') + ".csv"), 'w', newline='')
                self.csv_writers[topic] = csv.writer(self.csv_files[topic])
                self.csv_writers[topic].writerow(["eta.1", "eta.2", "eta.3"])
                self.create_subscription(std_msgs.msg.Float32MultiArray, topic, self.eta_callback, 10)
            elif topic == "/CSEI/state/psi":
                self.csv_files[topic] = open(os.path.join(self.csv_directory, topic.replace('/', '_') + ".csv"), 'w', newline='')
                self.csv_writers[topic] = csv.writer(self.csv_files[topic])
                self.csv_writers[topic].writerow(["psi"])
                self.create_subscription(std_msgs.msg.Float32, topic, self.psi_callback, 10)
            elif topic == "/CSEI/thrusters/tunnel/command":
                self.csv_files[topic] = open(os.path.join(self.csv_directory, topic.replace('/', '_') + ".csv"), 'w', newline='')
                self.csv_writers[topic] = csv.writer(self.csv_files[topic])
                self.csv_writers[topic].writerow(["force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"])
                self.create_subscription(geometry_msgs.msg.Wrench, topic, self.tunnel_callback, 10)
            elif topic == "/CSEI/thrusters/starboard/command":
                self.csv_files[topic] = open(os.path.join(self.csv_directory, topic.replace('/', '_') + ".csv"), 'w', newline='')
                self.csv_writers[topic] = csv.writer(self.csv_files[topic])
                self.csv_writers[topic].writerow(["force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"])
                self.create_subscription(geometry_msgs.msg.Wrench, topic, self.starboard_callback, 10)
            elif topic == "/CSEI/thrusters/port/command":
                self.csv_files[topic] = open(os.path.join(self.csv_directory, topic.replace('/', '_') + ".csv"), 'w', newline='')
                self.csv_writers[topic] = csv.writer(self.csv_files[topic])
                self.csv_writers[topic].writerow(["force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"])
                self.create_subscription(geometry_msgs.msg.Wrench, topic, self.port_callback, 10)

    def get_bag_info(self, bag_path):
        topics = subprocess.check_output(["ros2", "bag", "info", bag_path])
        topics = topics.decode("utf-8")
        pattern = r"Topic: (.*?) \| Type: (.*?) \|"
        matches = re.findall(pattern, topics)
        return {topic: type_str for topic, type_str in matches}
    
    
    def observer_callback(self, msg):
        data = [msg.eta[0], msg.eta[1], msg.eta[2], msg.nu[0], msg.nu[1], msg.nu[2], msg.bias[0], msg.bias[1], msg.bias[2]]
        self.csv_writers["/CSEI/observer/state"].writerow(data)

    def control_callback(self, msg):
        data = [msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]]
        self.csv_writers["/CSEI/control/u_cmd"].writerow(data)

    def tau_callback(self, msg):
        data = [msg.data[0], msg.data[1], msg.data[2]]
        self.csv_writers["/CSEI/state/tau"].writerow(data)

    def eta_callback(self, msg):
        data = [msg.data[0], msg.data[1], msg.data[2]]
        self.csv_writers["/CSEI/state/eta"].writerow(data)

    def psi_callback(self, msg):
        data = [msg.data]
        self.csv_writers["/CSEI/state/psi"].writerow(data)

    def tunnel_callback(self, msg):
        data = [msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]
        self.csv_writers["/CSEI/thrusters/tunnel/command"].writerow(data)

    def starboard_callback(self, msg):
        data = [msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]
        self.csv_writers["/CSEI/thrusters/starboard/command"].writerow(data)

    def port_callback(self, msg):
        data = [msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]
        self.csv_writers["/CSEI/thrusters/port/command"].writerow(data)

    def shutdown(self):
        for file in self.csv_files.values():
            file.close()
        super().shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DataExtractor()
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
