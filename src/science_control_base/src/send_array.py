import serial
import struct
import time
import rospy
from std_msgs.msg import Int32MultiArray

class ScienceSend():
    def __init__(self):
        rospy.init_node('sciencesendarray')
        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
        print(self.ser.is_open)
        rospy.Subscriber("/science_joy", Int32MultiArray, self.callback)
        # self.rate = rospy.Rate(2)
        rospy.spin()
        self.motor_vel_to_esp = []

    def callback(self, data):
        try:
            
                if data.data[1]==1:
                    self.motor_vel_to_esp = [1,50,30,150,40,60]
                    # self.motor_vel_to_esp = data.data


                    for value in self.motor_vel_to_esp:
                        self.ser.write(value.to_bytes(2, 'big'))
                        print(f"Sent Value : {value}")

                print(self.ser.read_all())

                # read = self.ser.readline().decode()
                # print(read)
                # self.rate.sleep()
                
                # data. = [42, 123, 789, 50, 40, 50, 115]
                # for value in values:
                #     self.ser.write(value.to_bytes(2, 'big'))  # Sending 2-byte integers
                #     # print(f"Sent Value: {value}")

                # read = self.ser.readline().decode()
                # print(read)


        except KeyboardInterrupt:
            self.serial_port.close()

if __name__ == '__main__':
    obj= ScienceSend()