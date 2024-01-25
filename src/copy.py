#!/usr/bin/env python

## Code for AutEx IRC 2024 
## 


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
        # im_sub = message_filters.Subscriber('/camera/color/image_raw', Image,)
        # dep_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        # self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 100, 0.01)
        # self.timeSynchronizer.registerCallback(self.image_callback)

        self.im_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback1)
        self.dep_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.image_callback2)

        self.left_cascade = cv2.CascadeClassifier('haar_trained_xml/left/cascade.xml')
        self.right_cascade = cv2.CascadeClassifier('haar_trained_xml/right/cascade.xml')
        self.motor_pub = rospy.Publisher('/motor_vals', Int32MultiArray, queue_size=10)

        self.midpoint_left = (320,240)
        self.midpoint_right = (320,240)

        # self.rate = rospy.Rate(20)
        self.flag =  0
        self.greenflag1 = 0
        self.greenflag2 = 0
        self.test_flag = 1
        

    def pubSpeed(self, jack, distance =15):
        self.leftwheelforw = 0
        self.leftwheelback = 0
        self.rightwheelforw = 0
        self.rightwheelback = 0

        

        if (jack == 1) & (200 < distance <1500):
            self.leftwheelforw = 50
            self.leftwheelback = 50
            self.rightwheelforw = 50
            self.rightwheelback = 50

        

        elif(jack == 1) & (10< distance <=200):
            self.leftwheelforw = 0
            self.leftwheelback = 0
            self.rightwheelforw = 0
            self.rightwheelback = 0
            motor_vals_array = [self.leftwheelforw, self.leftwheelback, self.rightwheelforw, self.rightwheelback]
            data_to_send = Int32MultiArray()
            data_to_send.data = motor_vals_array
            self.motor_pub.publish(data_to_send)
            rospy.sleep(20)

        elif (jack == 2, 10<distance<=200) :
            # keep turning left until arrow is detected agin
            self.leftwheelforw = 0
            self.leftwheelback = 0
            self.rightwheelforw = 20
            self.rightwheelback = 20

            # green flag is set because after turning for a while the arrow will be out of sight
            #    and jack2 will become unset, hence no further scanning of arrows
            self.greenflag1 = 1
        
            
            # incomplete

        elif (jack == 3, 10<distance<=200) :

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


        if (jack == 10):
            self.leftwheelforw = -20
            self.leftwheelback = -20
            self.rightwheelforw = 20
            self.rightwheelback = 20
            
            test_time = 5
            print("here")
            start = time.time()

        motor_vals_array = [self.leftwheelforw, self.leftwheelback, self.rightwheelforw, self.rightwheelback]
        data_to_send = Int32MultiArray()
        data_to_send.data = motor_vals_array
        self.motor_pub.publish(data_to_send)
        while (1):
            if(time.time()-start >5):
                print("fulfilled")
                break
            
            # publish


    def image_callback2(self, depth_data):
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough') 

        self.distance_left_arrow = self.depth_image[self.midpoint_left[0], self.midpoint_left[1]]
        self.distance_right_arrow = self.depth_image[self.midpoint_right[0], self.midpoint_right[1]]



        



    def image_callback1(self, image_data):
        self.image = self.bridge.imgmsg_to_cv2(image_data, 'bgr8')
        # self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough') 
        self.image = cv2.resize(self.image, (640,480))

        ###reduce the field of view to remove potential noise from trees and rocks

        self.image  = cv2.rectangle(self.image , (0,0), (640,200), (0,0,0), -1) 

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
        if len(left_signs):
            print (str(left_signs) + "-- left arrow")
        elif len(right_signs):
            print (str(right_signs) + "-- right arrow")


        # self.midpoint_left= sign_detector.show_box(self.image, left_signs)
        # self.midpoint_right= sign_detector.show_box(self.image , right_signs)

        # print(type(a))

        cv2.imshow("arrow detect", self.image)
        cv2.waitKey(100)

        if len(left_signs):
                
                x1, y1, w1, h1 = left_signs
                self.midpoint_left = ((x1+(x1+w1))/2, y1+(y1+h1)/2)
           
            
                print("left arrow being detected ")


                # distance_left_arrow = self.depth_image[midpoint_left[0], midpoint_left[1]]
                # distance_left_arrow = distance_left_arrow/10
                # self.pubSpeed(1, distance_left_arrow)
                # self.pubSpeed(2,distance_left_arrow)
        
        elif len(right_signs):
            # x2  = right_signs[0]
            # y2 = right_signs[1]
            # if (0<x2<=640) and (0<y2<=480):
                x2, y2, w2, h2 = right_signs
                self.midpoint_left = ((x2+(x2+w2))/2, y2+(y2+h2)/2)
               

                print("right arrow being detected ")
                # distance_right_arrow = self.depth_image[midpoint_right[0], midpoint_right[1]]
                # distance_right_arrow = distance_right_arrow/10
                # self.pubSpeed(1, distance_right_arrow)
                # self.pubSpeed(3, distance_right_arrow)

        else :
                print("no arrow being detected")


if __name__=='__main__':
    
    Autonomous = autonomous()
    # if Autonomous.greenflag1 ==1:
    #     Autonomous.pubSpeed(5)
    #     Autonomous.greenflag1 = 0

    # if Autonomous.greenflag2 ==1:
    #     Autonomous.pubSpeed(4)
    #     Autonomous.greenflag2 = 0

    if Autonomous.test_flag == 1:
        Autonomous.pubSpeed(10)


    if Autonomous.flag==0:
        rospy.spin()

