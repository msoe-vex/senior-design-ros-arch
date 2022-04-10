import rclpy
import numpy as np
import cv2
import torch
from collections import defaultdict
from rclpy.node import Node
from std_msgs.msg import String
from rospy_message_converter import message_converter

# TODO: Move NMS and state classification to project and fix imports
from utils.general import non_max_suppression
from platform_state import platform_state_with_determinism
from goal_state import is_goal_tipped

class CoreCVProcessing(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.color = None
        self.depth = None
        self.field_representation = defaultdict(list)
        
        # Calculation for object distance based on bounding box dimensions in inches
        self.FOCAL_LENGTH = ((448.0-172.0) * (24.0*0)) / (11.0)
        # Width of game objects in inches
        self.LABELS = {0:'Blue Goal', 1:'Blue Robot', 2:'Neutral Goal', 3:'Platform', 4:'Red Goal', 5:'Red Robot', 6:'Ring'}
        self.OBJECT_WIDTH_DICT = {"6":3.5, "0":12.5, "2":12.5, "4":12.5, "1":5.5, "5":5.5, "3":53.0}

        self.color_subscription = self.create_subscription(
            String,
            'camera/color/image_raw',
            self.color_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            String,
            'camera/depth/image_rect_raw',
            self.depth_callback,
            10)

    def color_callback(self, msg):
        # TODO: Determine format of color image message
        self.color = msg.data
        self.process_frame()

    def depth_callback(self, msg):
        # TODO: Obtain format depth data message
        self.depth = msg.data
        self.process_frame()

    def obj_distance(self, obj, depth_frame):   
        # object parsing
        x1, y1, x2, y2, _, obj_cls = obj

        # calculating distance using trigonometric properties
        trig_distance = (self.OBJECT_WIDTH_DICT[str(int(obj_cls))] * self.FOCAL_LENGTH)/(x2-x1) 
        
        # calculating center of object
        x = (x1 + x2)/2 
        y = (y1 + y2)/2
        
        # extract average distance from depth map
        center = depth_frame.get_distance(x, y)
        right = depth_frame.get_distance(x+2, y)
        top = depth_frame.get_distance(x, y+2)
        left = depth_frame.get_distance(x-2, y)
        bottom = depth_frame.get_distance(x, y-2)
        pixels = [center, right, top, left, bottom]
        points = float(sum([1 for x in pixels if x > 0]))
        depth_distance_inches = (center if center is not None else 0 +\
                                 right if right is not None else 0 +\
                                 top if top is not None else 0 +\
                                 left if left is not None else 0 +\
                                 bottom if bottom is not None else 0)/points
        
        # weighting and combining localization methods
        distance = (trig_distance * .2) + (depth_distance_inches * .8) 
        
        # in the event that depthmap can't detect distance, only use trig distance
        if (depth_distance_inches == 0):
            distance = trig_distance
        
        return distance

    def process_frame(self):
        if self.color is not None and self.depth is not None:
            # TODO: Verify whether this RGB to BGR color conversion is still needed
            # color_image = cv2.cvtColor(np.ascontiguousarray(self.color.get_data()), cv2.COLOR_RGB2BGR)

            # Format color frame and move to GPU
            color_image = np.ascontiguousarray(color_frame.get_data()).transpose((2, 0, 1))
            torch.cuda.synchronize()
            model_input = torch.from_numpy(color_image).cuda().half()
            model_input /= 255
            model_input = model_input[None]
            torch.cuda.synchronize()

            # Run model inference
            results = model(model_input)[0]
            torch.cuda.synchronize()

            # Run NMS algorithm
            nms_results = non_max_suppression(results, conf_thres=0.5)[0]
            torch.cuda.synchronize()

            for obj in nms_results:
                dist = float(obj_distance(obj, depth_frame).cpu())
                x1, y1, x2, y2, conf, obj_cls = obj.cpu()
                # TODO: Publish object and its distance to the 'cv_camera_tf/image' topic
                # TODO: Subscribe to the 'cv_camera_tf/coordinates' topic and store the transformed object coordinates
                pose_x, pose_y = None
                if obj_cls == 0.0:
                    # TODO: Validate the properties of the parameters (RGB | BGR)
                    state = is_goal_tipped(color_image, x1, y1, x2, y2)
                    blue_goal = {'x': pose_x, 'y': pose_y, 'c': 'blue', 's': state}
                    self.field_representation['goals'].append(blue_goal)
                elif obj_cls == 1.0:
                    blue_robot = {'x': pose_x, 'y': pose_y}
                    self.field_representation['blue_robots'].append(blue_robot)
                elif obj_cls == 2.0:
                    # TODO: Validate the properties of the parameters
                    state = is_goal_tipped(color_image, x1, y1, x2, y2)
                    neutral_goal = {'x': pose_x, 'y': pose_y, 'c': 'neutral', 's': state}
                    self.field_representation['goals'].append(neutral_goal)
                elif obj_cls == 3.0:
                    # TODO: Retrieve robot location from odometry node and validate the properties of the parameters
                    robot_location = None
                    color_int, state = platform_state_with_determinism(robot_location, color_image, x1, y1, x2, y2)
                    color = None
                    if color_int != -1:
                        color = 'blue' if color_int == 0 else 'red'
                    platform = {'x': pose_x, 'y': pose_y, 'c': color}
                    self.field_representation['platforms'].append(platform)
                elif obj_cls == 4.0:
                    # TODO: Validate the properties of the parameters
                    state = is_goal_tipped(color_image, x1, y1, x2, y2)
                    red_goal = {'x': pose_x, 'y': pose_y, 'c': 'red', 's': state}
                    self.field_representation['goals'].append(red_goal)
                elif obj_cls == 5.0:
                    red_robot = {'x': pose_x, 'y': pose_y}
                    self.field_representation['red_robots'].append(red_robot)
                elif obj_cls == 6.0:
                    ring = {'x': pose_x, 'y': pose_y}
                    self.field_representation['rings'].append(ring)
                
            # TODO: publish message to the 'cv_rep_packing' topic
            message = message_converter.convert_dictionary_to_ros_message(dict(self.field_representation))

def main(args=None):
    rclpy.init(args=args)
    core_cv_processor = CoreCVProcessing()
    rclpy.spin(core_cv_processor)
    core_cv_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
