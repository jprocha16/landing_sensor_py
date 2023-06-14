#!/usr/bin/env python3

import rospy
import cv2
import math
import numpy as np
from landing_sensor_py.msg import ArucoInfo
from geometry_msgs.msg import Point, Pose
from mavros_msgs.msg import VFR_HUD

'''
class PoseEstimation:
    """WILL BE PARENT CLASS POSE ESTIMATORS"""
    def __init__(self, node_name="pose_estimation", arucos_topic="uav/marker/arucos"):
        rospy.init_node(node_name, anonymous=True)

        # Subscribers
        self.info_sub = rospy.Subscriber(uav/marker/arucos, ArucoInfo, self.aruco_cb)


    def aruco_cb(self, msg):
        

    def run(self):
        rospy.spin()
'''


class ArucoPoseEstimation:
    def __init__(self, k, d, node_name="pose_estimation", arucos_topic="/uav/marker/arucos", heading_topic="/mavros/vfr_hud"):
        rospy.init_node(node_name, anonymous=True)

        # Subscribers
        self.aruco_info_sub = rospy.Subscriber(arucos_topic, ArucoInfo, self.aruco_cb)
        self.heading_sub = rospy.Subscriber(heading_topic, VFR_HUD, self.heading_cb)

        # Publishers
        self.translation_vec_pub = rospy.Publisher('/uav/marker/position', Point, queue_size=1)
        self.rotation_vec_pub = rospy.Publisher('/uav/marker/orientation', Point, queue_size=1)
        self. corrected_position_pub = rospy.Publisher('uav/marker/corrected_position', Point, queue_size=1)

        self.arucoid_1 = 129
        self.arucoid_2 = 280
        self.arucoid_1_size = 0.45     # 0.50
        self.arucoid_2_size = 0.05   # 0.0555
        self.k = k
        self.d = d
        self.heading_rad = 0

    def aruco_cb(self, msg):
        # Reformat id and corners
        corners = np.array([[msg.c1_x, msg.c1_y], [msg.c2_x, msg.c2_y], [msg.c3_x, msg.c3_y], [msg.c4_x, msg.c4_y]],
                           dtype=float)
        id = msg.id

        # Calls pose_estimation
        self.estimate_pose(id, np.reshape(corners, (1, 4, 2)))
        
    def heading_cb(self, data):
        # convert heading from degrees to radians
        degrees = data.heading
        rads = degrees * math.pi / 180
        self.heading_rad = rads

    def estimate_pose(self, ids, corners):
        if ids == self.arucoid_1:
            rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners, self.arucoid_1_size, self.k,
                                                                            self.d)
            self.pose_converter(tvec)
            
        elif ids == self.arucoid_2:
            rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners, self.arucoid_2_size, self.k,
                                                                            self.d)
            self.pose_converter(tvec)

        translation_vec = Point()
        translation_vec.x = tvec[0, 0, 0]
        translation_vec.y = tvec[0, 0, 1]
        translation_vec.z = tvec[0, 0, 2]
        self.translation_vec_pub.publish(translation_vec)

        rotation_vec = Point()
        rotation_vec.x = rvec[0, 0, 0]
        rotation_vec.y = rvec[0, 0, 1]
        rotation_vec.z = rvec[0, 0, 2]
        self.rotation_vec_pub.publish(rotation_vec)

    def pose_converter(self, tvec):
        raw = np.array([[-1 * tvec[0, 0, 0]],
                        [tvec[0, 0, 1]]])

        height = np.array([tvec[0, 0, 2]])
        # print(raw)
        # print(height)

        rot_matrix = np.array([[math.cos(self.heading_rad), math.sin(self.heading_rad)],
                               [-1 * math.sin(self.heading_rad), math.cos(self.heading_rad)]])
        # print(rot_matrix)

        corrected_xy = np.dot(rot_matrix, raw)
        # print(corrected_xy)

        corrected_xyz = np.vstack([corrected_xy, height])
        print(corrected_xyz)
        print(self.heading_rad)
        print("-------------------")
        
        corrected_position = Point()
        corrected_position.x = corrected_xyz[0, 0]
        corrected_position.y = corrected_xyz[1, 0]
        corrected_position.z = corrected_xyz[2, 0]
        self.corrected_position_pub.publish(corrected_position)


def main():
    # calibration_matrix_path = "/home/ciafa/catkin_ws/src/landing_sensor_py/scripts/calibration_matrix.npy"
    # distortion_coefficients_path = "/home/ciafa/catkin_ws/src/landing_sensor_py/scripts/distortion_coefficients.npy"
    # k = np.load(calibration_matrix_path)
    # d = np.load(distortion_coefficients_path)

    k = np.array([[999.461170663331, 0, 642.2582577578172], [0, 996.9611451866272, 474.1471906434548], [0, 0, 1]])
    d = np.array([0.1644274736100891, -0.2717244716038656, -0.002867946281892625, -9.69782173585606e-05, 0])
    pose_estimator = ArucoPoseEstimation(k, d)
    rospy.spin()


if __name__ == '__main__':
    main()
