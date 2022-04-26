import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rospy_message_converter import message_converter

from .fieldRepresentation import FieldRepresentation
from .platforms import BluePlatform, PlatformState, RedPlatform
from .scoring_elements import LowNeutralGoal, RedGoal, BlueGoal, Ring
from .robots import HostRobot, PartnerRobot, OpposingRobot
from .mathUtils import Pose2D
from .enumerations import Color

class RepPackingSubscriber(Node):
    def __init__(self):
        super().__init__('rep_packing_subscriber')
        self.field_representation = None
        self.field_view = None
        self.red_platform = RedPlatform(PlatformState.LEVEL)
        self.blue_platform = BluePlatform(PlatformState.LEVEL)
        self.team_color = None

        self.color_subscription = self.create_subscription(
            String,
            'robot/current_color',
            self.color_callback,
            10)

        self.CV_subscription = self.create_subscription(
            String,
            'cv_rep_packing',
            self.CV_callback,
            10)
        
    def color_callback(self, msg):
        if msg.data == "RED":
            self.team_color = Color.RED
        elif msg.data == "BLUE":
            self.team_color = Color.BLUE
        
    def CV_callback(self, msg):
        self.field_view = message_converter.convert_ros_message_to_dictionary(msg.data)
        if self.team_color and self.field_view:
            ring_arr = []
            goal_arr = []
            robot_arr = [HostRobot(team_color, Pose2D(self.field_view['pose_x'], self.field_view['pose_y']))]
            
            if 'rings' in self.field_view:
                for ring in self.field_view['rings']:
                    ring_arr.append(Ring(Pose2D(ring['x'], ring['y'])))
            
            if 'goals' in self.field_view:
                for goal in self.field_view['goals']:
                    t = False if goal['s'] < 1 else True
                    if goal['c'] == 'red':
                        goal_arr.append(RedGoal(Pose2D(goal['x'], goal['y']), tipped=t))
                    elif goal['c'] == 'blue':
                        goal_arr.append(BlueGoal(Pose2D(goal['x'], goal['y']), tipped=t))
                    elif goal['c'] == 'neutral':
                        goal_arr.append(LowNeutralGoal(Pose2D(goal['x'], goal['y']), tipped=t))

            if 'platforms' in self.field_view:
                for platform in self.field_view['platforms']:
                    plat_state = platform['s']
                    plat_color = platform['c']
                    platform_state = None

                    if plat_state == 0:
                        platform_state = PlatformState.LEFT
                    elif plat_state == 1:
                        platform_state = PlatformState.LEVEL
                    elif plat_state == 2:
                        platform_state = PlatformState.RIGHT
                    
                    if plat_state != -1:
                        if plat_color == 'blue':
                            self.blue_platform = BluePlatform(platform_state)
                        elif plat_color == 'red':
                            self.red_platform = RedPlatform(platform_state)
            
            if self.team_color == Color.RED:
                if 'blue_robots' in self.field_view:
                    for robot in self.field_view['blue_robots']:
                        robot_arr.append(OpposingRobot(Color.BLUE, Pose2D(robot['x'], robot['y'])))
                if 'red_robots' in in self.field_view:
                    partner = self.field_view['red_robots'][0] # Assuming only one partner robot
                    robot_arr.append(PartnerRobot(Color.RED, Pose2D(partner['x'], partner['y'])))
            elif self.team_color == Color.BLUE:
                if 'red_robots' in self.field_view:
                    for robot in self.field_view['red_robots']:
                        robot_arr.append(OpposingRobot(Color.RED, Pose2D(robot['x'], robot['y'])))
                if 'blue_robots' in in self.field_view:
                    partner = self.field_view['blue_robots'][0] # Assuming only one partner robot
                    robot_arr.append(PartnerRobot(Color.BLUE, Pose2D(partner['x'], partner['y'])))

            self.field_representation = FieldRepresentation(
                rings=ring_arr,
                goals=goal_arr,
                red_platform=red_platform,
                blue_platform=blue_platform,
                robots=robot_arr
            )
            
            # TODO: Add publishing code for self.field_representation to the 'field_rep/update' topic
        
def main(args=None):
    rclpy.init(args=args)
    rep_packing_subscriber = RepPackingSubscriber()
    rclpy.spin(rep_packing_subscriber)
    rep_packing_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()