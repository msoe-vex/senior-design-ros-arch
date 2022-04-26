import cv2 as cv
import numpy as np

def platform_state_with_determinism(robot_location, img, x1, y1, x2, y2):
    """
    This function takes in the image, bounding box, and robot position.
    Determines the state of the platform (left, center, right) and which color platform (blue, red)

    :param robot_location: (x, y, theta) of robot's location on field (meters, degrees)
    :param img: Full images from camera
    :param x1 y1 x2 y2: Object location identified by YOLOv5 model
    :return: integer tuple (color, state)
             color: (-1,0,1) -> (unknown, blue, red)
             state: (-1,0,1,2) -> (unknown, left, center, right) 
    """ 
    # Center 50% of the field
    # Only angles up to 30 degrees off center
    if robot_location[1] >= 1 and robot_location[1] <= 2.75:
        if abs(robot_location[2]) <= 60:
            # Blue Platform State
            plat_state = determine_platform_state(img, x1, y1, x2, y2)
            return 0,plat_state
        elif abs(robot_location[2]) >= 120:
            # Red Platform State
            plat_state = determine_platform_state(img, x1, y1, x2, y2)
            return 1,plat_state
    return -1,-1


def determine_platform_state(img, x1, y1, x2, y2): 
    """
    This function takes in the image and bounding box and determines the state of the platform (left, center, right)

    :param img: Full images from camera
    :param x1 y1 x2 y2: Object location identified by YOLOv5 model
    :return: returns -1 if state of platform cannot be determined
             returns one integer (0,1,2) corresponding to (left, center, right) state
    """ 
    # Resize image to only located platform object
    img = img[y1:y2, x1:x2]

    # Transform image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray, 75, 150)
    lines = cv.HoughLinesP(edges, 1, np.pi/180, 75, minLineLength=((x2-x1) * 0.6), maxLineGap=90)
    bins = [0,0,0]

    # Bin line slopes
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2-y1)/(x2-x1)
            if slope <= -0.3:
                bins[0] += 1
            elif slope >= 0.3:
                bins[2] += 1
            else:
                bins[1] += 1
            cv.line(img, (x1, y1), (x2, y2), (0, 0, 128), 1)

        # Determine direction of platform based on bins
        if bins[0] > bins[2]:
            # Platform Left
            return 0
        elif bins[2] > bins[0]:
            # Platform Right
            return 2
        else:
            # Platform Center
            return 1
    else:
        # State Unknown
        return -1