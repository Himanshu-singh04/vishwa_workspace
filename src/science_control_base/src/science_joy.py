#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor

class ScienceJoy():
    def __init__(self):
       self.pub=rospy.Publisher('/science_joy',Int32MultiArray,queue_size = 10)
       self.wormgear1_pwm = 0
       self.wormgear2_pwm = 0
       self.johnson_pwm = 0
       self.servo_pwm_shake = 0
       self.pump1 = 0 
       self.pump2 = 0    

       self.sub = rospy.Subscriber("/joy", Joy, self.callback)
   
    def callback(self, data):
        

               ###########  Wormgear1 -- Scoop Mechnaism ####
       if(data.axes[7] == 1):
               self.wormgear1_pwm = 1
       if(data.axes[7] == -1):
               self.wormgear1_pwm = 2
       if(data.axes[7] == 0):
               self.wormgear1_pwm = 0
               
               ###########  Wormgear2 -- Scoop Mechnaism ####

       if(data.axes[6] == 1):
               self.wormgear2_pwm = 1
       if(data.axes[6] == -1):
               self.wormgear2_pwm = 2
       if(data.axes[6] == 0):
               self.wormgear2_pwm = 0

               ############Johnson Motor to Drill##############

       if(data.buttons[5]==1) :
               self.johnson_pwm = 1
       if(data.buttons[4]==1) :
               self.johnson_pwm = 2  # turns backward 
       if (data.buttons[5]==0 and data.buttons[4]==0):
               self.johnson_pwm = 0
        

               ############## Pump pair 1 -- Deposits Chemicals#######
       if(data.buttons[1]==1) :
               self.pump1 = 1  
       if (data.buttons[1]==0):
               self.pump1 = 0

               ############## Pump pair2  -- Deposits Chemicals#######

       if(data.buttons[3]==1) :
               self.pump2 = 1  
       if (data.buttons[3]==0):
               self.pump2 = 0

               #######3######  Servo shake -- Mixes the chemical

       if(data.buttons[2]==1) :
               self.servo_pwm_shake = 1  
       if (data.buttons[2]==0 ):
               self.servo_pwm_shake = 0


       array = [self.johnson_pwm, self.wormgear1_pwm , self.wormgear2_pwm,
                                 self.servo_pwm_shake, 
                                              self.pump1, self.pump2 ]
        
       array_to_send = Int32MultiArray()
       array_to_send.data = array
       self.pub.publish(array_to_send)
       print(array_to_send.data)
       print("published")

    
def main():
    rospy.init_node('ScienceJoy')
    science = ScienceJoy()
    print("first loop looped")
    rospy.spin()

if __name__ == '__main__':
    main()