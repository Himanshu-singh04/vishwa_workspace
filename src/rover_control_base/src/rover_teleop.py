#!/usr/bin/env python3
import rospy
# from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor

class WheelState:
    def __init__(self) -> None:
        self.wheel_1_r = 0
        self.wheel_2_l = 0
        self.wheel_3_l = 0
        self.wheel_4_r = 0
        # self.ang_wheel_1 = 0
        # self.ang_wheel_2 = 0
        # self.ang_wheel_3 = 0
        # self.ang_wheel_4 = 0
        self.val = 0

        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, data):
        # Case 1 : onspot rotation
        if(data.buttons[0]==1): # digital key to set 45 degree position
            # self.ang_wheel_1 = -0.78539816339744830961566084581988
            # self.ang_wheel_2 = 0.78539816339744830961566084581988
            # self.ang_wheel_3 = -0.78539816339744830961566084581988
            # self.ang_wheel_4 = 0.78539816339744830961566084581988
            self.val = 2

            if(data.axes[0]>0): # left command rotation
                self.wheel_1_r = -floor(50*data.axes[0])
                self.wheel_2_l = floor(50*data.axes[0])
                self.wheel_3_l = floor(50*data.axes[0])
                self.wheel_4_r = -floor(50*data.axes[0])
                
            elif(data.axes[0]<0): # right command rotation
                self.wheel_1_r = -floor(50*data.axes[0])
                self.wheel_2_l = floor(50*data.axes[0])
                self.wheel_3_l = floor(50*data.axes[0])
                self.wheel_4_r = -floor(50*data.axes[0])

        # Case 2 : Default position        
        elif(data.buttons[3]==1): 
            # self.ang_wheel_1 = 0
            # self.ang_wheel_2 = 0
            # self.ang_wheel_3 = 0
            # self.ang_wheel_4 = 0
            self.val = 0
            self.wheel_1_r = 0
            self.wheel_2_l = 0
            self.wheel_3_l = 0
            self.wheel_4_r = 0

        # Case 3 : zigzag motion
        elif(data.axes[2]==-1 and data.axes[5]==-1):
            if(data.axes[6] == -1): # digital key for angular control enable
                # self.ang_wheel_2 = 0
                # self.ang_wheel_1 = 0
                # self.ang_wheel_3 = 0 
                # self.ang_wheel_4 = 0
                self.val = 1

            elif(data.axes[6] == 1): # digital key for angular control enable
                # self.ang_wheel_2 = 0
                # self.ang_wheel_1 = 0
                # self.ang_wheel_3 = 0 
                # self.ang_wheel_4 = 0
                self.val = -1

            elif(data.axes[4]>0):

                self.wheel_1_r = floor(50*data.axes[4])
                self.wheel_2_l = floor(50*data.axes[4])
                self.wheel_3_l = floor(50*data.axes[4])
                self.wheel_4_r = floor(50*data.axes[4])

            elif(data.axes[4]<0):

                self.wheel_1_r = floor(50*data.axes[4])
                self.wheel_2_l = floor(50*data.axes[4])
                self.wheel_3_l = floor(50*data.axes[4])
                self.wheel_4_r = floor(50*data.axes[4])

        # elif(data.buttons[2] == 1):
            # self.ang_wheel_1 = 100
            # self.ang_wheel_2 = 100
            # self.ang_wheel_3 = 100
            # self.ang_wheel_4 = 100
            # self.wheel_1_r = 100
            # self.wheel_2_l = 0
            # self.wheel_3_l = 0
            # self.wheel_4_r = 0

    # Case 4 : Default Differential 
        else:
            if(data.axes[3]<0): # right differential command

                self.wheel_1_r = floor((30*data.axes[3])*(1-data.axes[3]))
                self.wheel_2_l = -floor((30*data.axes[3])*(1+data.axes[3]))
                self.wheel_3_l = -floor((30*data.axes[3])*(1+data.axes[3]))
                self.wheel_4_r = floor((30*data.axes[3])*(1-data.axes[3]))
            
            elif(data.axes[3]>0): # left differential command

                self.wheel_1_r = floor((30*data.axes[3])*(1+data.axes[3]))
                self.wheel_2_l = -floor((30*data.axes[3])*(1-data.axes[3]))
                self.wheel_3_l = -floor((30*data.axes[3])*(1-data.axes[3]))
                self.wheel_4_r = floor((30*data.axes[3])*(1+data.axes[3]))

            elif(data.axes[1]>0): # forward command
                self.wheel_1_r = floor(50*data.axes[1])
                self.wheel_2_l = floor(50*data.axes[1])
                self.wheel_3_l = floor(50*data.axes[1])
                self.wheel_4_r = floor(50*data.axes[1])

            elif(data.axes[1]<0): # backward command
                self.wheel_1_r = floor(50*data.axes[1])
                self.wheel_2_l = floor(50*data.axes[1])
                self.wheel_3_l = floor(50*data.axes[1])
                self.wheel_4_r = floor(50*data.axes[1])

        wheel_vels_arr = [self.wheel_1_r , self.wheel_2_l , self.wheel_3_l , self.wheel_4_r]
        ang_arr = [self.val]
        wheel_data_to_send = Int32MultiArray()
        ang_data_to_send = Int32MultiArray()
        wheel_data_to_send.data = wheel_vels_arr
        ang_data_to_send.data = ang_arr
        pub1.publish(wheel_data_to_send)
        pub2.publish(ang_data_to_send)
        print(str(wheel_data_to_send.data) + "--- wheels speed")
        print(str(ang_data_to_send.data) + "--- wheel angles")

def main():
    rospy.init_node('joy_to_p2')
    wheel_state = WheelState()
    global pub1, pub2
    pub1=rospy.Publisher('wheel_vels',Int32MultiArray,queue_size = 10)
    pub2=rospy.Publisher('wheel_angs',Int32MultiArray,queue_size = 10)
    rospy.spin()
    

if __name__ == '__main__':
    main()