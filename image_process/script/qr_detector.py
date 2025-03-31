#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pytesseract

class OCRHazardLabelReader:
    def __init__(self):
        self.bridge = CvBridge()
        # ตั้งชื่อ node ของ ROS
        rospy.init_node('ocr_hazard_label_reader', anonymous=True)
        # Subscribe ภาพจาก camera topic
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        rospy.loginfo("OCR Hazard Label Reader Node Started")

    def image_callback(self, msg):
        try:
            # แปลง ROS Image message เป็น OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # แปลงภาพเป็นภาพเทาเพื่อให้การประมวลผล OCR ดีกว่า
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # ใช้การกรองเพื่อปรับคุณภาพภาพให้เหมาะสม
            ret, thresh = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

            # ใช้ pytesseract ในการแยกข้อความจากภาพ
            extracted_text = pytesseract.image_to_string(thresh, lang='eng')

            # ถ้าพบข้อความ
            if extracted_text:
                rospy.loginfo("Detected Text: %s", extracted_text)

                # แสดงข้อความที่อ่านได้จาก OCR
                cv2.putText(cv_image, extracted_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            # แสดงผลภาพที่มีข้อความ OCR
            cv2.imshow("OCR Hazard Label Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ocr_reader = OCRHazardLabelReader()
        ocr_reader.start()
    except rospy.ROSInterruptException:
        pass
