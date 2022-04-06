# Package dependencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Package code
class Publisher(Node):
    def __init__(self):
        # Call into the parent node to create the name of this node
        super().__init__('publisher')

        # Create a publisher with an assigned "topic" to send data to
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Define how long we want to spend between messages
        timer_period = 0.5  # seconds

        # Call the "create_timer" function from the Node class, which will execute
        # the callback function "timer_callback" every time "timer_period" elapses
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the counter to 0
        self.i = 0

    def timer_callback(self):
        # Create a string to hold our message
        msg = String()

        # Populate the message with the string we want
        msg.data = 'Hello World: %d' % self.i

        # Call the publisher in order to send our message to the ROS network
        self.publisher_.publish(msg)

        # Log what we published
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Increment our instance counter
        self.i += 1


def main(args=None):
    # Initialize the ROS node
    rclpy.init(args=args)

    # Create a new publisher instance
    publisher = Publisher()

    # "Spin" to update the nodes
    rclpy.spin(publisher)

    # Explicitly destroy the node (similar to a deconstructor - optional here)
    publisher.destroy_node()

    # Shut down the ROS node
    rclpy.shutdown()


if __name__ == '__main__':
    main()