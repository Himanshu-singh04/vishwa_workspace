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
        self.bevel1 = 0
        self.bevel2 = 0
        self.base = 0
        self.link_1 = 0
        self.link_2 = 0
        # self.val = 0

        self.count = 0
        self.prev = 0
        # self.curr = 0
        # self.diff = 0

        rospy.Subscriber("/joy2", Joy, self.callback)

    def callback(self, data):
        
        self.link_1 = floor(255*data.axes[1]) 
                
        self.base = floor(255*data.axes[0]) 

        self.link_2 = floor(255*data.axes[4])
      
        if (data.buttons[1]==1):
            self.claw = 255*data.buttons[1]
        elif (data.buttons[2]==1):
            self.claw = -255*data.buttons[2]
        else:
            self.claw = 0

        # if (data.axes[6] == -1):
        #     self.base = 0
        #     self.link_1 = 0
        #     self.link_2 = 0
        #     self.bevel1 = 0
        #     self.bevel2 = 0
        #     self.claw = 0

    ###  curr_bevel_value = bev
        # if (data.axes[2] == -1):
        #     self.bevel1 = 155
        #     self.bevel2 = 155
            
        # elif (data.axes[5] == -1): 
        #     self.bevel1 = -155
        #     self.bevel2 = -155

        # else:
        #     self.bevel1 = 0
        #     self.bevel2 = 0
        
        if(data.axes[3]>0 ):
            self.bevel2 = floor(100*data.axes[3])
            self.bevel1 = -floor(100*data.axes[3])

        elif(data.axes[3]<0):
            self.bevel2 = floor(100*data.axes[3])
            self.bevel1 = -floor(100*data.axes[3])
        else:
            self.bevel1=0
            self.bevel2=0
        # arm1_arr=[self.claw,self.clawanti,self.bevel,self.bevelanti]
        arm_arr = [self.base, self.link_1 ,self.link_2 , self.bevel1, self.bevel2, self.claw]
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