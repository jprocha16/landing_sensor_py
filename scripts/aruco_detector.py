#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import cv_bridge
import cv2
import numpy as np
from landing_sensor_py.msg import ArucoInfo



ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

'''
class MarkerDetector:
    """WILL BE PARENT CLASS OF MARKER DETECTORS"""
    def __init__(self, node_name="marker_detector", image_topic="sensor_images", preprocessor=None):
        rospy.init_node(node_name, anonymous=True)

        # Subscribers
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_cb)

        self.preprocessor = preprocessor

    def image_cb(self, msg: Image):
        im_pre = self.preprocessor.process(msg)
        detector_outputs = self.detect(im_pre)
        # calls function that publishes output
        # self.detector_publish(detector_output)
        raise NotImplemented

    def detector_publish(self, ):
        raise NotImplemented

    def run(self):
        rospy.spin()
'''


class PreProcessor:
    def __init__(self, calibration_matrix: np.ndarray, distortion_coefs: np.ndarray):
        self.calibration_matrix = calibration_matrix
        self.distortion_coefs = distortion_coefs

    def process(self, im_msg: Image):
        bridge = cv_bridge.CvBridge()
        opencv_image = bridge.imgmsg_to_cv2(im_msg, desired_encoding="bgr8")

        gray = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2GRAY)
        return gray


class ArucoDetector:
    def __init__(self, node_name="marker_detector", image_topic="/sensor_images", preprocessor: PreProcessor = None):
        rospy.init_node(node_name, anonymous=True)

        # Subscribers
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_cb)

        self.preprocessor = preprocessor

        # Publishers
        # TODO: criar mensagem
        # ArucoInfo == corners & ids
        self.aruco_info_pub = rospy.Publisher('/uav/marker/arucos', ArucoInfo, queue_size=1)

        # TODO: no futuro, ler isto a partir de um ficheiro xml
        # setup aruco dict
        self.aruco_dict_type = "DICT_7X7_1000"
        cv2.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_dict_type])

        self.parameters = cv2.aruco.DetectorParameters_create()

        self.aruco_id_1 = 129
        self.aruco_id_2 = 280

    def image_cb(self, msg: Image):
        im_pre = self.preprocessor.process(msg)
        self.process(im_pre)

    def process(self, im: np.ndarray):
        corners_lst, ids, rejected_img_points = cv2.aruco.detectMarkers(im, cv2.aruco_dict, parameters=self.parameters)
        # corners is tuple
        # ids is numpy array
        # print(corners_lst)

        seq = 0
        # publish corners and id
        for i, corners in enumerate(corners_lst):
            if ids[i][0] == self.aruco_id_1 or ids[i][0] == self.aruco_id_2:
                corners_id = ids[i][0]
                msg = ArucoInfo()
                msg.header.seq = seq
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "frame_id"
                seq += 1

                msg.id = corners_id

                msg.c1_x = int(corners[0, 0, 0])
                msg.c1_y = int(corners[0, 0, 1])
                msg.c2_x = int(corners[0, 1, 0])
                msg.c2_y = int(corners[0, 1, 1])
                msg.c3_x = int(corners[0, 2, 0])
                msg.c3_y = int(corners[0, 2, 1])
                msg.c4_x = int(corners[0, 3, 0])
                msg.c4_y = int(corners[0, 3, 1])

                self.aruco_info_pub.publish(msg)


def main():
    calibration_matrix_path = "/home/ciafa/catkin_ws/src/landing_sensor_py/scripts/calibration_matrix.npy"
    distortion_coefficients_path = "/home/ciafa/catkin_ws/src/landing_sensor_py/scripts/distortion_coefficients.npy"

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)
    preprocessor = PreProcessor(calibration_matrix=k, distortion_coefs=d)
    aruco_detector = ArucoDetector(preprocessor=preprocessor)
    rospy.spin()


if __name__ == '__main__':
    main()
