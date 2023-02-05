#!/usr/bin/env python3
import struct
import can
import rospy 
import numpy as np
from canfd_protocol import CANProtocol
from geometry_msgs.msg import Twist
from math import pi

class ROStoCANFD():
    def __init__(self, bustype):

        rospy.init_node('publish_vel')

        ## init param
        channel = rospy.get_param('~channel')
        bitrate = rospy.get_param('~bitrate')
        data_bitrate = rospy.get_param('~data_bitrate')
        self.timeout = rospy.get_param('~msg_timeout')
        print("Channel:\t", channel)
        print("Bitrate:\t", bitrate)
        print("Data bitrate:\t", data_bitrate)
        print("Msg timeout:\t", self.timeout)
        
        self.lengthWheelBase=0.64
        self.radius= 0.116
        self.rpm_max=3600
        self.coeff_riduz= 43.25

        ## init Bus interface
        self.bus = can.interface.Bus(channel = channel, bustype = bustype, fd = True, bitrate = bitrate, data_bitrate = data_bitrate)

        try:
            rospy.Subscriber("my_cmd_vel", Twist, self.callback)
        except:
            rospy.logerr("Nodo my_cmd_vel")
        
        rospy.spin()
      

    def callback(self,data):
        
        message = self.bus.recv(1.0)
        if message is not None:

            device_id = CANProtocol.get_device_id(message.arbitration_id)
            message_id = CANProtocol.get_message_id(message.arbitration_id)

            while message_id != CANProtocol.GET_TWIST_CMD():
                message = self.bus.recv(1.0)
                if message is not None:

                    device_id = CANProtocol.get_device_id(message.arbitration_id)
                    message_id = CANProtocol.get_message_id(message.arbitration_id)
    
            if message_id == CANProtocol.GET_TWIST_CMD():

                if CANProtocol.check_integrity(message.data[0:], CANProtocol.GET_TWIST_CMD_LEN()):
        
                    # Get linear velocities v_x and v_y
                    v_x = data.linear.x
                    # Get angular velocity on z
                    angular_vel_z = data.angular.z

                    # Calculate wheel velocities
                    w_r = (2*v_x + angular_vel_z*self.lengthWheelBase) / (2*self.radius)
                    w_l = (2*v_x - angular_vel_z*self.lengthWheelBase) / (2*self.radius)
                
                    # Calculate wheel rotations per minute
                    rpm_r = (w_r * self.coeff_riduz * 60) / (2 * pi)
                    rpm_l = (w_l * self.coeff_riduz * 60) / (2 * pi)
                    if(rpm_r>self.rpm_max):
                        rpm_r=self.rpm_max
                    if(rpm_l>self.rpm_max):
                        rpm_l=self.rpm_max

                    payload = bytearray() 


                    ## make msg to send
                    payload.extend(CANProtocol.MASTER_ID().to_bytes(1, 'little'))
                    payload.extend(message.data[1:2])

                    payload.extend(struct.pack('h', int(rpm_r)))
                    payload.extend(struct.pack('h', int(rpm_l)))
                                    
                    msg_data = CANProtocol.create_message(payload)
                    msg = can.Message(
                                        is_fd = True,
                                        bitrate_switch = True,
                                        is_extended_id = False,
                                        arbitration_id = (CANProtocol.MPU_ID() << 6) + CANProtocol.GET_TWIST_ACK(),
                                        data = msg_data
                                    )
                    self.bus.send(msg)

                    return
    



        
       
                       
def main():
    print("#########################")
    print("#   PUBLISH AMCL VEL    #")
    print("#########################")
    ros_to_can = ROStoCANFD('socketcan')

if __name__ == '__main__':
    main()

