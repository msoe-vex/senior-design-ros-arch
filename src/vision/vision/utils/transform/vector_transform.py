import numpy as np

# Intel Realsense specific horizontal and vertical field of view
HFOV, VFOV = 86, 57


class Quaternion():
    def __init__(self, qx: float, qy: float, qz: float, qw: float):
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
    
    def __str__(self):
        return "Quaternion: [{0}, {1}, {2}, {3}]".format(round(self.qx, 3), round(self.qy, 3), round(self.qz, 3), round(self.qw, 3))


def getQuaternionFromEuler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Convert an Euler angle to a quaternion.

    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return Quaternion(qx, qy, qz, qw)

def rotDiff(q1: Quaternion,q2: Quaternion) -> Quaternion:
    """Finds the quaternion that, when applied to q1, will rotate an element to q2"""
    conjugate = Quaternion(q2.qx*-1,q2.qy*-1,q2.qz*-1,q2.qw)
    return rotAdd(q1,conjugate)

def rotAdd(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """Finds the quaternion that is the equivalent to the rotation caused by both input quaternions applied sequentially."""
    w1 = q1.qw
    w2 = q2.qw
    x1 = q1.qx
    x2 = q2.qx
    y1 = q1.qy
    y2 = q2.qy
    z1 = q1.qz
    z2 = q2.qz

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

    return Quaternion(x,y,z,w)

def Quat2Mat(q: Quaternion) -> np.array:
    """Converts a quaternion to a rotation matrix (Numpy Array)"""
    m00 = 1 - 2 * q.qy**2 - 2 * q.qz**2
    m01 = 2 * q.qx * q.qy - 2 * q.qz * q.qw
    m02 = 2 * q.qx * q.qz + 2 * q.qy * q.qw
    m10 = 2 * q.qx * q.qy + 2 * q.qz * q.qw
    m11 = 1 - 2 * q.qx**2 - 2 * q.qz**2
    m12 = 2 * q.qy * q.qz - 2 * q.qx * q.qw
    m20 = 2 * q.qx * q.qz - 2 * q.qy * q.qw
    m21 = 2 * q.qy * q.qz + 2 * q.qx * q.qw
    m22 = 1 - 2 * q.qx**2 - 2 * q.qy**2
    result = [[m00,m01,m02],[m10,m11,m12],[m20,m21,m22]]

    return np.array(result)

def coordTransform(M: np.array, A: np.array, translation: np.array) -> np.array:
    """
    :param M: rotation matrix that represents the rotation between frames (q1 -> q2)
    :param A: vector of interest in frame q1
    :param translation: distance vector between origins of q1 and q2
    :return: vector A in relation to frame q2
    """
    APrime = np.matmul(M, A)
    q2Vec = APrime + translation
    return q2Vec

def determine_camera_vector(x1, y1, x2, y2, dist):
    """
    :param x1 y1 x2 y2: Object location identified by YOLOv5 model
    :param dist: Object distance from camera (inches)
    :return: vector of object location from camera
    """
    x = float((x1 + x2)/2.0)
    y = float((y1 + y2)/2.0)
    h_angle = np.radians(((x - 320.0)/(320.0))*(HFOV/2)) #TODO - verfiy calculations and output
    v_angle = np.radians(((y - 320.0)/(320.0))*(VFOV/2)) #TODO - verfiy calculations and output
    
    # Convert polar angles into vector
    v_x = dist * np.cos(v_angle) * np.cos(h_angle)
    v_y = dist * np.cos(v_angle) * np.sin(h_angle)
    v_z = dist * np.sin(v_angle)
    vec = np.array([v_x, v_y, v_z])
    return vec

class FrameTransform():
    def __init__(self):
        self.camera = getQuaternionFromEuler(0,np.radians(10),0) #TODO - update  rotational changes in camera frame (roll-X, pitch-Y, yaw-Z)
        self.robot = Quaternion(0.0,0.0,0.0,1.0)
        self.cameraToRobotTranslation = np.array([3.25,0.0,4.5]) #TODO - update translational changes in camera frame (X,Y,Z)
        self.gyroDistanceToGround = 0 # TODO - update actual value based on gyro distance to ground

    def get_object_location(self, x1, y1, x2, y2, dist, robot_location):
        """
        :param x1 y1 x2 y2: Object location identified by YOLOv5 model
        :param dist: Object distance from camera (inches)
        :param robot_location: (x, y, theta) of robot's location on field (inches, degrees)
        :return: (x,y,z) vector of objects location on field
        """

        #Calculate vector from camera to robot
        cameraToRobotRotation = Quat2Mat(rotDiff(self.camera, self.robot))
        print("Camera Vector: ", determine_camera_vector(x1, y1, x2, y2, dist))
        vec1 = coordTransform(cameraToRobotRotation, determine_camera_vector(x1, y1, x2, y2, dist), self.cameraToRobotTranslation)
        print("Robot Vector: ", vec1)
        
        # Calculate vector from robot to field
        field = getQuaternionFromEuler(0.0, 0.0, -np.radians(robot_location[2]))
        robotToFieldRotation = Quat2Mat(rotDiff(self.robot, field))
        robotToFieldTranslation = np.array([robot_location[0], robot_location[1], self.gyroDistanceToGround])
        vec2 = coordTransform(robotToFieldRotation, vec1, robotToFieldTranslation)

        return vec2




# Testing main - TODO remove
def main():
    camera = getQuaternionFromEuler(0,np.radians(15),0) #TODO - update for actual rotational changes in camera frame (0.2618 rad)
    robot = Quaternion(0.0,0.0,0.0,1.0)
    cameraToRobotTranslation = np.array([-0.19,0,0.3]) #TODO - update for actual translational changes in camera frame (X,Y,Z)

    cameraToRobotRotation = Quat2Mat(rotDiff(camera, robot))
    vec1 = coordTransform(cameraToRobotRotation, np.array([1,1,0]), cameraToRobotTranslation) #TODO - just a test vector (actual = vec in profile_test.py)
    print(vec1)

    # Temp robot location and rotation values for testing (x, y, theta)
    # TODO - update for actual robot location at time of image capture
    robotLocation = (2, 2, -135)

    field = getQuaternionFromEuler(0.0, 0.0, -np.radians(robotLocation[2]))
    robotToFieldRotation = Quat2Mat(rotDiff(robot, field))
    robotToFieldTranslation = np.array([robotLocation[0], robotLocation[1], 0.0]) # TODO - update actual z value based on gyro distance to ground
    vec2 = coordTransform(robotToFieldRotation, vec1, robotToFieldTranslation)
    print(vec2)

if __name__ == '__main__':
    main()
