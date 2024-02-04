#!/usr/bin/env python3
import rospy
# from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor
import pygame

pygame.init()

class WheelState:
    def __init__(self):
        self.pub1=rospy.Publisher('wheel_vels',Int32MultiArray,queue_size = 10)
        # self.pub2=rospy.Publisher('wheel_angs',Int32MultiArray,queue_size = 10)
        self.wheel_1_r = 0
        self.wheel_2_l = 0
        self.wheel_3_l = 0
        self.wheel_4_r = 0
        # self.val = 0

        # rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self):
        clock = pygame.time.Clock()
        joysticks = {}
        print("inside cb")
        done = False
        while not done:
                # Event processing step.
                # Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
                # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
                for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                                done = True  # Flag that we are done so we exit this loop.

                if event.type == pygame.JOYBUTTONDOWN:
                        print("Joystick button pressed.")
                        if event.button == 0:
                                joystick = joysticks[event.instance_id]

                if event.type == pygame.JOYBUTTONUP:
                        print("Joystick button released.")

                # Handle hotplugging
                if event.type == pygame.JOYDEVICEADDED:
                        # This event will be generated when the program starts for every
                        # joystick, filling up the list without needing to create them manually.
                        joy = pygame.joystick.Joystick(event.device_index)
                        joysticks[joy.get_instance_id()] = joy
                        print(f"Joystick {joy.get_instance_id()} connencted")

                if event.type == pygame.JOYDEVICEREMOVED:
                        del joysticks[event.instance_id]
                        print(f"Joystick {event.instance_id} disconnected")

        joy1= joysticks[0]
        joy2= joysticks[1]

        if(joy1.axis[3]<0): # right differential command

                self.wheel_1_r = -floor((200*joy1.axes[3]))
                self.wheel_2_l = floor((200*joy1.axes[3]))
                self.wheel_3_l = floor((200*joy1.axes[3]))
                self.wheel_4_r = -floor((200*joy1.axes[3]))
            
        if(joy1.axes[3]>0): # left differential command

                self.wheel_1_r = -floor((200*joy1.axes[3]))
                self.wheel_2_l = floor((200*joy1.axes[3]))
                self.wheel_3_l = floor((200*joy1.axes[3]))
                self.wheel_4_r = -floor((200*joy1.axes[3]))

        if(joy1.axes[1]>0): # forward command
                self.wheel_1_r = floor(155*joy1.axes[1])
                self.wheel_2_l = floor(155*joy1.axes[1])
                self.wheel_3_l = floor(155*joy1.axes[1])
                self.wheel_4_r = floor(155*joy1.axes[1])

        if(joy1.axes[1]<0): # backward command
                self.wheel_1_r = floor(155*joy1.axes[1])
                self.wheel_2_l = floor(155*joy1.axes[1])
                self.wheel_3_l = floor(155*joy1.axes[1])
                self.wheel_4_r = floor(155*joy1.axes[1])

        if(joy2.button[0]):
              print("drill on")
        else: 
              print("drill off")
        #drill binary
        
        wheel_vels_arr = [self.wheel_1_r , self.wheel_2_l , self.wheel_3_l , self.wheel_4_r]
        wheel_data_to_send = Int32MultiArray()
        wheel_data_to_send.data = wheel_vels_arr
        self.pub1.publish(wheel_data_to_send)
        print(str(wheel_vels_arr) + "--- wheels speed")

        clock.tick(30)        
def main():
    rospy.init_node('joy_to_p2')
#     joysticks = {}
#     joysticks = pygame.joystick.get_count()
    wheel_state = WheelState()
    wheel_state.callback()
    rospy.spin()
    

if __name__ == '__main__':
    main()
    pygame.quit()

# for joystick in joysticks.values():
        #     jid = joystick.get_instance_id()

        #     # Get the name from the OS for the controller/joystick.
        #     name = joystick.get_name()
        #     guid = joystick.get_guid()
        #     power_level = joystick.get_power_level()
        #     # Usually axis run in pairs, up/down for one, and left/right for
        #     # the other. Triggers count as axes.
        #     axes = joystick.get_numaxes()

        #     for i in range(axes):
        #         axis = joystick.get_axis(i)

        #     buttons = joystick.get_numbuttons()

        #     for i in range(buttons):
        #         button = joystick.get_button(i)

        #     hats = joystick.get_numhats()

        #     # Hat position. All or nothing for direction, not a float like
        #     # get_axis(). Position is a tuple of int values (x, y).
        #     for i in range(hats):
        #         hat = joystick.get_hat(i)
