#!/usr/bin/env python

## Code for AutEx IRC 2024 
## Maintainer - Furquan Shiekh


import rospy
import cv2 as cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import message_filters
import sign_detector
from std_msgs.msg import Int32MultiArray

import time 

class autonomous():
    def __init__(self):
        rospy.init_node("autonomous")
        self.bridge = CvBridge()
        im_sub = message_filters.Subscriber('/camera/color/image_raw', Image,)
        dep_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 100, 0.01)
        self.timeSynchronizer.registerCallback(self.image_callback)

        self.left_cascade = cv2.CascadeClassifier('haar_trained_xml/left/cascade.xml')
        self.right_cascade = cv2.CascadeClassifier('haar_trained_xml/right/cascade.xml')
        self.motor_pub = rospy.Publisher('/motor_vals', Int32MultiArray, queue_size=10)

        self.rate = rospy.Rate(20)
        self.flag =  0
        self.greenflag1 = 0
        self.greenflag2 = 0

    def pubSpeed(self, jack ,distance):
        self.leftwheelforw = 0
        self.leftwheelback = 0
        self.rightwheelforw = 0
        self.rightwheelback = 0

        if (jack == 1) & (1500< distance <=200):
            self.leftwheelforw = 50
            self.leftwheelback = 50
            self.rightwheelforw = 50
            self.rightwheelback = 50

        elif(jack == 1) & (10< distance <200):
            self.leftwheelforw = 0
            self.leftwheelback = 0
            self.rightwheelforw = 0
            self.rightwheelback = 0
            motor_vals_array = [self.leftwheelforw, self.leftwheelback, self.rightwheelforw, self.rightwheelback]
            data_to_send = Int32MultiArray()
            data_to_send.data = motor_vals_array
            self.motor_pub.publish(data_to_send)
            rospy.sleep(20)

        elif (jack == 2, 10<distance<200) :
            # keep turning left until arrow is detected agin
            self.leftwheelforw = 0
            self.leftwheelback = 0
            self.rightwheelforw = 20
            self.rightwheelback = 20

            # green flag is set because after turning for a while the arrow will be out of sight
            #    and jack2 will become unset, hence no further scanning of arrows
            self.greenflag1 = 1
        
            
            # incomplete

        elif (jack == 3, 10<distance<200) :

            # keep turning left until arrow is detected agin
            self.leftwheelforw = 20
            self.leftwheelback = 20
            self.rightwheelforw = 0
            self.rightwheelback = 0
            self.greenflag2 =1     ## for uninterrupted scanning 

           

        elif (jack == 4):
            self.leftwheelforw = 20
            self.leftwheelback = 20
            self.rightwheelforw = 0
            self.rightwheelback = 0

        elif (jack == 5):
            self.leftwheelforw = 0
            self.leftwheelback = 0
            self.rightwheelforw = 20
            self.rightwheelback = 20


        motor_vals_array = [self.leftwheelforw, self.leftwheelback, self.rightwheelforw, self.rightwheelback]
        data_to_send = Int32MultiArray()
        data_to_send.data = motor_vals_array
        self.motor_pub.publish(data_to_send)



    def image_callback(self, image_data, depth_data):
        self.image = self.bridge.imgmsg_to_cv2(image_data, 'bgr8')
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough') 
        self.gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        self.gray = cv2.resize(self.gray, (640,480), interpolation=cv2.INTER_AREA)
        self.gray = cv2.medianBlur(self.gray, 5)
        kernel = np.ones((5,5),np.uint8)
        self.grayp = cv2.erode(self.gray, kernel, iterations = 3)
        self.grayp = cv2.dilate(self.grayp, kernel, iterations = 2)
        self.edges = cv2.Canny(self.gray,100,150,apertureSize = 3)

        # cv2.imshow("current state", self.edges)
        # cv2.waitKey(100) 

        left_signs = sign_detector.classify_signs(self.edges, self.left_cascade)
        right_signs = sign_detector.classify_signs(self.edges, self.right_cascade)
        # print((left_signs))
        # print((right_signs))


        midpoint_left= sign_detector.show_box(self.edges, left_signs)
        midpoint_right= sign_detector.show_box(self.edges, right_signs)

        # print(type(a))

        cv2.imshow("arrow detect", self.edges)
        cv2.waitKey(100)

        if len(left_signs):
            print("left arrow being detected ")
            distance_left_arrow = self.depth_image[midpoint_left[0], midpoint_left[1]]
            self.pubSpeed(1, distance_left_arrow)
            self.pubSpeed(2,distance_left_arrow)
        
        elif len(right_signs):
            print("right arrow being detected ")
            distance_right_arrow = self.depth_image[midpoint_right[0], midpoint_right[1]]
            self.pubSpeed(1, distance_right_arrow)
            self.pubSpeed(3, distance_right_arrow)

        if self.greenflag1 == 1 :
            self.pubSpeed(5)
            self.greenflag1 = 0

        if self.greenflag2 == 1:
            self.pubSpeed(4)
            self.greenflag2 = 0


if __name__=='__main__':
    
    Autonomous = autonomous()
    if Autonomous.flag==0:                    #yeh kyun kiya
        rospy.spin()

