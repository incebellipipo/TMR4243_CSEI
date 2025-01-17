#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library
import rclpy.node  # Node class from ROS 2 Python client library
import std_msgs.msg  # Standard ROS 2 message types

# Define a class that inherits from rclpy.node.Node
class StringPublisherSubscriber(rclpy.node.Node):
    def __init__(self):
        # Initialize the node with the name 'publish_text'
        super().__init__('publish_text')

        # Create a publisher that will publish String messages to the '/chatter' topic
        self.pub = self.create_publisher(std_msgs.msg.String, '/chatter', 10)

        # Create a subscription to the '/chatter' topic, with the callback method 'callback'
        self.sub = self.create_subscription(std_msgs.msg.String, '/chatter', self.callback, 10)

    # Method to publish a message
    def publish_message(self):
        # Create a new String message
        msg = std_msgs.msg.String()
        # Set the data of the message
        msg.data = 'hi from python'
        # Publish the message
        self.pub.publish(msg)

    # Callback method that gets called when a message is received on the '/chatter' topic
    def callback(self, msg):
        # Log the received message
        self.get_logger().info('I heard: "%s"' % msg.data)

# Main function
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the PublishString node
    node = StringPublisherSubscriber()

    # Publish a message
    node.publish_message()

    # Keep the node running to listen for messages
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    # Shutdown the rclpy library
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
