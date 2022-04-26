import rclpy
import numpy as np
import cv2
import torch
from collections import defaultdict
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rospy_message_converter import message_converter

from utils.general import non_max_suppression
from utils.transform.vector_transform import FrameTransform
from utils.state.platform_state import platform_state_with_determinism
from utils.state.goal_state import is_goal_tipped
from model.detect import non_max_suppression
from model.experimental import attempt_load

class CoreCVProcessing(Node):
    def __init__(self):
        super().__init__('core_cv_processor')
        self.color = None
        self.depth = None
        self.pose = None
        self.tf2 = FrameTransform()
        self.bridge = CvBridge()
        self.model = attempt_load('static/best.pt', map_location=torch.device('cuda:0'))
        self.field_representation = defaultdict(list)
        self.rep_publisher = self.create_publisher(String, 'cv_rep_packing', 10)
        
        # Initializing model
        self.model.model.half()
        self.model.forward(torch.zeros((1, 3, 480, 640)).to(torch.device('cuda:0')).type(torch.half))
        
        # Calculation for object distance based on bounding box dimensions in inches
        self.FOCAL_LENGTH = ((448.0-172.0) * (24.0)) / (11.0)
        # Width of game objects in inches
        self.LABELS = {0:'Blue Goal', 1:'Blue Robot', 2:'Neutral Goal', 3:'Platform', 4:'Red Goal', 5:'Red Robot', 6:'Ring'}
        self.OBJECT_WIDTH_DICT = {"6":3.5, "0":12.5, "2":12.5, "4":12.5, "1":5.5, "5":5.5, "3":53.0}

        self.color_subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.color_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            'camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        self.odom_subscription = self.create_subscription(
            String,
            'robot/odometry/get_pose',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        # Current assumption position of format: int dict w/ x, y, t[heta]
        self.pose = message_converter.convert_ros_message_to_dictionary(msg.data)
        self.process_frame()

    def color_callback(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg.data, desired_encoding='passthrough')
        self.process_frame()

    def depth_callback(self, msg):
        # Rectified depth image, contains float depths in meters
        self.depth = self.bridge.imgmsg_to_cv2(msg.data, desired_encoding='passthrough')
        self.process_frame()

    def obj_distance(self, obj, depth_frame):   
        # object parsing
        x1, y1, x2, y2, _, obj_cls = obj

        # calculating distance using trigonometric properties
        trig_distance = (self.OBJECT_WIDTH_DICT[str(int(obj_cls))] * self.FOCAL_LENGTH)/(x2-x1) 
        
        # calculating center of object
        x = (x1 + x2)/2 
        y = (y1 + y2)/2
        
        # Extract average distance from depth map
        center = self.depth[x, y] * 39.3700787402
        right = self.depth[x+2, y] * 39.3700787402
        top = self.depth[x, y+2] * 39.3700787402
        left = self.depth[x-2, y] * 39.3700787402
        bottom = self.depth[x, y-2] * 39.3700787402
        pixels = [center, right, top, left, bottom]
        points = float(sum([1 for x in pixels if x > 0]))
        depth_distance_inches = (center if center is not None else 0 +\
                                 right if right is not None else 0 +\
                                 top if top is not None else 0 +\
                                 left if left is not None else 0 +\
                                 bottom if bottom is not None else 0)/points
        
        # weighting and combining localization methods
        distance = (trig_distance * .15) + (depth_distance_inches * .85) 
        
        # in the event that depthmap can't detect distance, only use trig distance
        if (depth_distance_inches == 0):
            distance = trig_distance
        
        return distance

    def process_frame(self):
        if self.color is not None and self.depth is not None and self.pose is not None:
            # Format color frame and move to GPU
            # TODO: Verify whether this conversion is still needed
            color_image = np.ascontiguousarray(self.color).transpose((2, 0, 1))
            torch.cuda.synchronize()
            model_input = torch.from_numpy(color_image).cuda().half()
            model_input /= 255
            model_input = model_input[None]
            torch.cuda.synchronize()

            # Run model inference
            results = self.model(model_input)[0]
            torch.cuda.synchronize()

            # Run NMS algorithm
            nms_results = non_max_suppression(results, conf_thres=0.5)[0]
            torch.cuda.synchronize()

            # Calculates the distance of all game objects in frame
            for obj in nms_results:
                dist = float(self.obj_distance(obj, self.depth).cpu())
                x1, y1, x2, y2, conf, obj_cls = obj.cpu()
                robot_location = (self.pose['x'], self.pose['y'], self.pose['t'])
                object_location = self.tf2.get_object_location(x1, y1, x2, y2, dist, robot_location)
                pose_x, pose_y = object_location[0], object_location[1]

                # Adds object to field representation
                if obj_cls == 0.0:
                    state = is_goal_tipped(color_image, x1, y1, x2, y2)
                    blue_goal = {'x': pose_x, 'y': pose_y, 'c': 'blue', 's': state}
                    self.field_representation['goals'].append(blue_goal)
                elif obj_cls == 1.0:
                    blue_robot = {'x': pose_x, 'y': pose_y}
                    self.field_representation['blue_robots'].append(blue_robot)
                elif obj_cls == 2.0:
                    state = is_goal_tipped(color_image, x1, y1, x2, y2)
                    neutral_goal = {'x': pose_x, 'y': pose_y, 'c': 'neutral', 's': state}
                    self.field_representation['goals'].append(neutral_goal)
                elif obj_cls == 3.0:
                    color_int, state = platform_state_with_determinism((self.pose['x'], self.pose['y'], self.pose['t']), color_image, x1, y1, x2, y2)
                    color = None
                    if color_int != -1:
                        color = 'blue' if color_int == 0 else 'red'
                    platform = {'x': pose_x, 'y': pose_y, 'c': color}
                    self.field_representation['platforms'].append(platform)
                elif obj_cls == 4.0:
                    state = is_goal_tipped(color_image, x1, y1, x2, y2)
                    red_goal = {'x': pose_x, 'y': pose_y, 'c': 'red', 's': state}
                    self.field_representation['goals'].append(red_goal)
                elif obj_cls == 5.0:
                    red_robot = {'x': pose_x, 'y': pose_y}
                    self.field_representation['red_robots'].append(red_robot)
                elif obj_cls == 6.0:
                    ring = {'x': pose_x, 'y': pose_y}
                    self.field_representation['rings'].append(ring)
                
            self.field_representation['pose_x'] = self.pose['x']
            self.field_representation['pose_y'] = self.pose['y']
                
            # Publishing field representation to the 'cv_rep_packing' topic
            msg = String()
            msg.data = message_converter.convert_dictionary_to_ros_message(dict(self.field_representation))
            self.rep_publisher.publish(msg)
            
            # Reset frame buffer
            self.color = None  
            self.depth = None          

def main(args=None):
    rclpy.init(args=args)
    core_cv_processor = CoreCVProcessing()
    rclpy.spin(core_cv_processor)
    core_cv_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
