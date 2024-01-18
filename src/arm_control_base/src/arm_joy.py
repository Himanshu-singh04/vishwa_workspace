#!/usr/bin/env python3
import rospy
# from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor

class ArmState:
    def __init__(self):
        self.pub1=rospy.Publisher('arm',Int32MultiArray,queue_size = 10)

        self.claw = 0
        self.bevel = 0
        self.base = 0
        self.link_1 = 0
        self.link_2 = 0
        # self.val = 0

        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, data):
        
        if(data.axes[1]>0): 
            self.link_1 = floor(10*data.axes[1]) 

        elif(data.axes[1]<0):
            self.link_1 = floor(10*data.axes[1])
                
        elif(data.axes[0]>0):
            self.base = floor(10*data.axes[0]) 

        elif(data.axes[0]<0):
            self.base = floor(10*data.axes[0])

        elif(data.axes[4]>0): 
            self.link_2 = floor(10*data.axes[1])
                
        elif (data.axes[4]<0):
            self.link_2 = floor(10*data.axes[4])

        elif(data.axes[6]>0):
            self.bevel = floor(10*data.axes[6])

        elif(data.axes[6]<0):
            self.bevel = floor(10*data.axes[6])

        elif(data.buttons[0] == 1):
            self.claw = 1

        # arm_arr = [self.base,self.baseanti ,self.pwml1 , self.pwml2]
        # arm1_arr=[self.claw,self.clawanti,self.bevel,self.bevelanti]
        arm_arr = [self.base,self.link_1 ,self.link_2 , self.bevel, self.claw]
        arm_data_to_send = Int32MultiArray()
        # arm1_data_to_send = Int32MultiArray()
        arm_data_to_send.data = arm_arr
        # arm1_data_to_send.data = arm1_arr
        self.pub1.publish(arm_data_to_send)
        # pub2.publish(arm1_data_to_send)
        print(str(arm_data_to_send.data) + "--- arm")
        # print(str(arm1_data_to_send.data) + "--- arm1")

def main():
    rospy.init_node('joy_to_p2_p3')
    arm_state = ArmState()
    rospy.spin()
    

if __name__ == '__main__':
    main()