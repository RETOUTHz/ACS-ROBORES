#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HazardLabelDetector:
    def __init__(self):
        self.bridge = CvBridge()
        # ตั้งชื่อ node ของ ROS
        rospy.init_node('hazard_label_detector', anonymous=True)
        # Subscribe ภาพจาก camera topic
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        rospy.loginfo("Hazard Label Detection Node Started")
        
        # โหลดป้าย hazard label template (หากไม่มีให้ใช้ตัวอย่างหรือภาพที่มีป้าย hazard label)
        self.template = cv2.imread("/path/to/hazard_label_template.png", 0)  # ป้าย hazard label
        self.orb = cv2.ORB_create()

        # หาจุดสำคัญใน template
        self.kp_template, self.des_template = self.orb.detectAndCompute(self.template, None)

    def image_callback(self, msg):
        try:
            # แปลง ROS Image message เป็น OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # หาจุดสำคัญและ descriptors ในภาพที่ต้องการตรวจจับ
            kp_image, des_image = self.orb.detectAndCompute(gray_image, None)

            # ใช้ Brute Force Matcher เพื่อจับคู่ descriptor
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(self.des_template, des_image)

            # เรียงลำดับ matches โดยการหาค่าต่ำสุด
            matches = sorted(matches, key = lambda x:x.distance)

            # วาด matches บนภาพ
            result_image = cv2.drawMatches(self.template, self.kp_template, cv_image, kp_image, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

            # แสดงภาพที่มีการจับคู่
            cv2.imshow("Hazard Label Detection", result_image)
            cv2.waitKey(1)

            # แสดงจำนวน matches ใน terminal
            rospy.loginfo("Number of Matches: %d", len(matches))

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        hazard_detector = HazardLabelDetector()
        hazard_detector.start()
    except rospy.ROSInterruptException:
        pass
