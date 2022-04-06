# Project dependencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Project code
class Subscriber(Node):
    def __init__(self):
        # Call the constructor for the parent Node object with the node name
        super().__init__('subscriber')

        # Create a subscription to the topic "topic", linked to our callback
        # function "listener_callback"
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Call this to prevent a warning from it "being unused"
        self.subscription

    # Define our callback function
    def listener_callback(self, msg):
        # Log what was received
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    # Initialize the ROS node
    rclpy.init(args=args)

    # Create a new publisher instance
    minimal_subscriber = Subscriber()

    # "Spin" to update the nodes
    rclpy.spin(minimal_subscriber)

    # Explicitly destroy the node (similar to a deconstructor - optional here)
    minimal_subscriber.destroy_node()

    # Shut down the ROS node
    rclpy.shutdown()


if __name__ == '__main__':
    main()