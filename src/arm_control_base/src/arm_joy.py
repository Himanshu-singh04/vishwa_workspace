#!/usr/bin/env python3
import rospy
# from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor

class ArmState:
    def __init__(self) -> None:
        pub1=rospy.Publisher('arm',Int32MultiArray,queue_size = 10)
        pub2=rospy.Publisher('arm1',Int32MultiArray,queue_size = 10)
        self.claw = 0
        # self.all=0
        self.clawanti = 0
        self.bevel = 0
        self.bevelanti = 0
        self.base = 0
        self.baseanti = 0
        self.pwml1 = 0
        self.pwml2 = 0
        # would love to understand what in the pwm is going on here.
        
        # self.val = 0

        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, data):
        
        if(data.axes[1]>0 ): 
            self.pwml1 =floor(50*data.axes[1]) 
                
        if (data.axes[4]>0):
            self.pwml2 =floor(50*data.axes[4]) 

        elif(data.axes[1]<0 ): 
            self.pwml1 = floor(50*data.axes[1])
                
        elif ( data.axes[4]<0):
            self.pwml2 = floor(50*data.axes[4])

        elif(data.axes[6] == -1): 
            self.base=1

        elif(data.axes[6] == 1): 
            self.baseanti=1

        elif(data.buttons[1]==1):
             self.bevel=1

        elif(data.buttons[2]==1):
             self.bevelanti=1

        elif(data.buttons[0]==1):
             self.clawanti=1

        elif(data.buttons[3]==1):
             self.claw=1
        
        elif(data.buttons[7]==1):
            self.claw = 0
            # self.all = 1
            self.clawanti = 0
            self.bevel = 0
            self.bevelanti = 0
            self.base = 0
            self.baseanti = 0
            self.pwml1 = 0
            self.pwml2 = 0
        # self.val = 0

        # This should be a seperate publish fxn

        arm_arr = [self.base,self.baseanti ,self.pwml1 , self.pwml2]
        arm1_arr=[self.claw,self.clawanti,self.bevel,self.bevelanti]
        arm_data_to_send = Int32MultiArray()
        arm1_data_to_send = Int32MultiArray()
        arm_data_to_send.data = arm_arr
        arm1_data_to_send.data = arm1_arr
        self.pub1.publish(arm_data_to_send)
        self.pub2.publish(arm1_data_to_send)
        print(str(arm_data_to_send.data) + "--- arm")
        print(str(arm1_data_to_send.data) + "--- arm1")

def main():
    rospy.init_node('joy_to_p2_p3')
    arm_state = ArmState()
    # global pub1,pub2
    # using global variables increases coupling and is generally frowned 
    # upon. Pls read what encapsulation is.
    # additionally makes you look like a noob so... dekhle bhai
    rospy.spin()
    # exception and node shutdown handling expected here
    # rospy.shutdown()
    

if __name__ == '__main__':
    main()