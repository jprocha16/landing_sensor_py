#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from landing_sensor_py.msg import ArucoInfo
from geometry_msgs.msg import Point, Pose


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
    def __init__(self, k, d, node_name="pose_estimation", arucos_topic="/uav/marker/arucos"):
        rospy.init_node(node_name, anonymous=True)

        # Subscribers
        self.aruco_info_sub = rospy.Subscriber(arucos_topic, ArucoInfo, self.aruco_cb)

        # Publishers
        self.translation_vec_pub = rospy.Publisher('/uav/marker/position', Point, queue_size=1)
        self.rotation_vec_pub = rospy.Publisher('/uav/marker/orientation', Point, queue_size=1)
        # todo: trocar o formato das mensagens publicadas para Pose em vez de Point (converter para quaternion)
        # self.pose_pub = rospy.Publisher('uav/marker/pose', Pose, queue_size=1)

        self.arucoid_1 = 129
        self.arucoid_2 = 280
        self.arucoid_1_size = 0.45     # 0.50
        self.arucoid_2_size = 0.05   # 0.0555
        self.k = k
        self.d = d

    def aruco_cb(self, msg):
        # Reformat id and corners
        corners = np.array([[msg.c1_x, msg.c1_y], [msg.c2_x, msg.c2_y], [msg.c3_x, msg.c3_y], [msg.c4_x, msg.c4_y]],
                           dtype=float)
        id = msg.id

        # Calls pose_estimation
        self.estimate_pose(id, np.reshape(corners, (1, 4, 2)))

    def estimate_pose(self, ids, corners):
        if ids == self.arucoid_1:
            rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners, self.arucoid_1_size, self.k,
                                                                            self.d)
        elif ids == self.arucoid_2:
            rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners, self.arucoid_2_size, self.k,
                                                                            self.d)

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


def main():
    calibration_matrix_path = "/home/ciafa/catkin_ws/src/landing_sensor_py/scripts/calibration_matrix.npy"
    distortion_coefficients_path = "/home/ciafa/catkin_ws/src/landing_sensor_py/scripts/distortion_coefficients.npy"

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)
    pose_estimator = ArucoPoseEstimation(k, d)
    rospy.spin()


if __name__ == '__main__':
    main()
