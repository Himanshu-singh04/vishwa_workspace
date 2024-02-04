#!/usr/bin/env python3
import rospy
# from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor
import sys

class WheelState:
    def __init__(self):
        self.pub1=rospy.Publisher('wheel_vels',Int32MultiArray,queue_size = 10)
        self.pub2=rospy.Publisher('wheel_angs',Int32MultiArray,queue_size = 10)
        self.wheel_1_r = 0 #rear left
        self.wheel_2_l = 0 #rear right
        self.wheel_3_l = 0 #front right
        self.wheel_4_r = 0 #front left
        self.ang_wheel_1 = 0
        self.ang_wheel_2 = 0
        self.ang_wheel_3 = 0
        self.ang_wheel_4 = 0

        
        # self.val = 0

        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, data):
    # Case 4 : Default Differential 
        
        ##########    EMERGENCY KILL BRAKE  #########

        if(data.buttons[0]==1):
                self.wheel_1_r = 0
                self.wheel_2_l = 0
                self.wheel_3_l = 0
                self.wheel_4_r = 0
                wheel_vels_arr = [self.wheel_1_r , self.wheel_2_l , self.wheel_3_l , self.wheel_4_r]
                # ang_arr = [self.ang_wheel_1 , self.ang_wheel_2 , self.ang_wheel_3 , self.ang_wheel_4]
                wheel_data_to_send = Int32MultiArray()
                # ang_data_to_send = Int32MultiArray()
                wheel_data_to_send.data = wheel_vels_arr
                # ang_data_to_send.data = ang_arr
                self.pub1.publish(wheel_data_to_send)
                self.pub1.publish(wheel_data_to_send)
                self.pub1.publish(wheel_data_to_send)
                self.pub1.publish(wheel_data_to_send)
                self.pub1.publish(wheel_data_to_send)
                self.pub1.publish(wheel_data_to_send)
                print(str(wheel_data_to_send.data) + "--- wheels speed")
                rospy.signal_shutdown("emergency")

        
        if(data.axes[2]<0): # right differential command

                self.wheel_1_r = -floor((130*data.axes[2]))
                self.wheel_2_l = floor((130*data.axes[2]))
                self.wheel_3_l = floor((130*data.axes[2]))
                self.wheel_4_r = -floor((130*data.axes[2]))

        elif(data.axes[2]>0): # left differential command

                self.wheel_1_r = -floor((130*data.axes[2]))
                self.wheel_2_l = floor((130*data.axes[2]))
                self.wheel_3_l = floor((130*data.axes[2]))
                self.wheel_4_r = -floor((130*data.axes[2]))


        elif(data.axes[1]>0): # forward command
                self.wheel_1_r = floor(150*data.axes[1])
                self.wheel_2_l = floor(150*data.axes[1])
                self.wheel_3_l = floor(150*data.axes[1])
                self.wheel_4_r = floor(150*data.axes[1])

        elif(data.axes[1]<0): # backward command
                self.wheel_1_r = floor(150*data.axes[1])
                self.wheel_2_l = floor(150*data.axes[1])
                self.wheel_3_l = floor(150*data.axes[1])
                self.wheel_4_r = floor(150*data.axes[1])

        if(abs(self.wheel_1_r)<3 or abs(self.wheel_2_l)<3 or abs(self.wheel_3_l)<3 or abs(self.wheel_4_r)<3 ):
                self.wheel_1_r = 0
                self.wheel_2_l = 0
                self.wheel_3_l = 0
                self.wheel_4_r = 0

        wheel_vels_arr = [self.wheel_1_r , self.wheel_2_l , self.wheel_3_l , self.wheel_4_r]
        ang_arr = [self.ang_wheel_1 , self.ang_wheel_2 , self.ang_wheel_3 , self.ang_wheel_4]
        wheel_data_to_send = Int32MultiArray()
        ang_data_to_send = Int32MultiArray()
        wheel_data_to_send.data = wheel_vels_arr
        ang_data_to_send.data = ang_arr
        self.pub1.publish(wheel_data_to_send)
        self.pub2.publish(ang_data_to_send)
        print(str(wheel_data_to_send.data) + "--- wheels speed")
        print(str(ang_data_to_send.data) + "--- wheel angles")

def main():
    rospy.init_node('joy_to_p2')
    wheel_state = WheelState()

    rospy.spin()
    

if __name__ == '__main__':
    main()