#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoPublisher:
    def __init__(self, video_file):
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(video_file)
        self.pub = rospy.Publisher('sensor_images', Image, queue_size=10)

    def start(self):
        rate = rospy.Rate(30)  # 30 fps
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.pub.publish(msg)
                rate.sleep()
            else:
                break


if __name__ == '__main__':
    rospy.init_node('image_publisher')
    vp = VideoPublisher("/home/ciafa/ArUCo-Markers-Pose-Estimation-Generation-Python/Images/e_aruco_test_video.mp4")
    vp.start()
    rospy.spin()

