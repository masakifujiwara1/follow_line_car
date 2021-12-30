#! /usr/bin/env python

from numpy import maximum
import rospy
from rospy.rostime import Duration
from sensor_msgs.msg import Image
import cv2 as cv
import cv_bridge
from geometry_msgs.msg import Twist
import numpy as np

MAX_VEL = 0.1
left_anguler = 0.1
right_anguler = -0.1
big_num = 0.1
lx1, lx2 = 240, 250
ly1, ly2 = 300, 400
rx1, rx2 = 400, 410
ry1, ry2 = 300, 400


class follow_line:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv.namedWindow('window', 1)
        self.image_sub = rospy.Subscriber(
            'camera/rgb/image_raw', Image, self.image_callback)
        self.line_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')

        twist = Twist()
        twist.linear.x = MAX_VEL

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        lower_skin = np.array([0, 0, 100], dtype=np.uint8)
        upper_skin = np.array([180, 45, 255], dtype=np.uint8)

        mask = cv.inRange(hsv, lower_skin, upper_skin)
        cv.rectangle(image, (lx1, ly1), (lx2, ly2), (0, 0, 255), 1)
        cv.rectangle(image, (rx1, ry1), (rx2, ry2), (0, 0, 255), 1)

        left_part = mask[ly1:ly2, lx1:lx2]
        right_part = mask[ry1:ry2, rx1:rx2]

        count_L_part = cv.countNonZero(left_part)
        count_R_part = cv.countNonZero(right_part)

        if 100 >= abs(count_L_part - count_R_part) >= 0:
            twist.angular.z = 0
        elif count_L_part >= 950 and count_L_part > count_R_part:
            twist.angular.z = left_anguler + big_num * 2
        elif count_R_part >= 950 and count_R_part > count_L_part:
            twist.angular.z = right_anguler - big_num * 2
        elif count_L_part > 800 and count_L_part > count_R_part:
            twist.angular.z = left_anguler + big_num
        elif count_R_part > 800 and count_R_part > count_L_part:
            twist.angular.z = right_anguler - big_num
        elif count_L_part > 0 and count_L_part > count_R_part:
            twist.angular.z = left_anguler
        elif count_R_part > 0 and count_R_part > count_L_part:
            twist.angular.z = right_anguler
        elif count_R_part == 0 and count_L_part == 0:
            twist.angular.z = 0

        print("L:", count_L_part, "R:", count_R_part)

        cv.imshow('window', image)
        cv.waitKey(3)

        self.Publisher(twist)

    def Publisher(self, Twist):
        self.line_pub.publish(Twist)


rospy.init_node('follow_line_node')
follow_line_node = follow_line()
rospy.spin()
# if __name__ == '__main__':
#     rg = follow_line()
#     DURATION = 1
#     r = rospy.Rate(1 / DURATION)
#     while not rospy.is_shutdown():
#         rg.__init__()
#         r.sleep()
