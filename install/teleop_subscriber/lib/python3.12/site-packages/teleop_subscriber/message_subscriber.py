# Import the necessary ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Define a class for your subscriber node, inheriting from the ROS2 Node class
class MessageSubscriber(Node):

    def __init__(self):
        # Call the constructor of the parent Node class and give it a name
        super().__init__('simple_subscriber')

        # Create a subscriber. This is the core of the node's functionality.
        # self.create_subscription(msg_type, topic_name, callback_function, qos_profile)
        self.subscription = self.create_subscription(
            String,             # The message type (std_msgs.msg.String)
            'input',          # The topic name to subscribe to
            self.listener_callback, # The function to call when a message is received
            10)                 # The Quality of Service profile depth
        
        # This is just to prevent a "variable not used" warning
        self.subscription  

        # Log a message to the console indicating the node has started
        self.get_logger().info('Subscriber node started. Waiting for messages...')

    def listener_callback(self, msg):
        """
        This function is called every time a message is published to the 'chatter' topic.
        The message data is passed to this function as the 'msg' argument.
        """
        # Log the received message data to the console
        # The 'data' attribute of the String message holds the actual text
        data = msg.data
        self.get_logger().info(f'I heard: "{msg.data}"')
        if(data.lower() == "right"):
            self.get_logger().info('Sendin current to right motor')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the MessageSubscriber node
    message_subscriber = MessageSubscriber()

    # "Spin" the node, which means it will keep running and processing callbacks
    # until it's manually shut down (e.g., with Ctrl+C).
    rclpy.spin(message_subscriber)

    # When the node is shut down, destroy it and clean up
    message_subscriber.destroy_node()
    rclpy.shutdown()


# This is the standard entry point for a Python script
if __name__ == '__main__':
    main()