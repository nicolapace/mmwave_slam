#!/usr/bin/env python3
import struct
import can
import rospy , ros_numpy
import numpy as np
import time
from canfd_protocol import CANProtocol
from geometry_msgs.msg import Vector3 , Twist
from sensor_msgs.msg import PointCloud2
import tf

# from sklearn.cluster import DBSCAN



"""
Nodo per il solo movimento e scrittura delle velocità sul topic /cmd_vel per monitoring
"""
class ROStoCANFD():
    def __init__(self, bustype):

        rospy.init_node('ros_to_can')

        ## init param
        channel = rospy.get_param('~channel')
        bitrate = rospy.get_param('~bitrate')
        data_bitrate = rospy.get_param('~data_bitrate')
        self.timeout = rospy.get_param('~msg_timeout')
        print("Channel:\t", channel)
        print("Bitrate:\t", bitrate)
        print("Data bitrate:\t", data_bitrate)
        print("Msg timeout:\t", self.timeout)

        ## init Bus interface
        self.bus = can.interface.Bus(channel = channel, bustype = bustype, fd = True, bitrate = bitrate, data_bitrate = data_bitrate)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        try:
            rospy.Subscriber('/joypad', Vector3, self.read_joy_callback, queue_size=1)
        except:
            rospy.logerr("Nodo Pulsanti non attivo")
        
        vel = Twist()
        payload = bytearray()

        self.set_time = True 
        timer = 0
        
        self.v_prec_mod = 0
        self.UpLX_time = time.time()
        self.filtering = 0              # Filtro vel non attivo

        ## main loop
        while not rospy.is_shutdown():
            
            message = self.bus.recv(1.0)
            if message is not None:

                device_id = CANProtocol.get_device_id(message.arbitration_id)
                message_id = CANProtocol.get_message_id(message.arbitration_id)
        
                if message_id == CANProtocol.GET_TWIST_CMD():

                    if CANProtocol.check_integrity(message.data[0:], CANProtocol.GET_TWIST_CMD_LEN()):
                        payload = bytearray() 

                        ## extract Rset_velocity e Lset_velocity
                        Rset = struct.unpack('h', message.data[2:4])[0]
                        Lset = struct.unpack('h', message.data[4:6])[0]

                        ## calculate pseudo-velocity
                        v = ( Rset + Lset ) / 2.0
                        w = (Rset - Lset)
                        
                        v = self.filtering_velocity(v)

                        Rset = ( 2*v + w )/2
                        Lset = ( 2*v - w )/2
                        
                        ## make msg to send
                        payload.extend(CANProtocol.MASTER_ID().to_bytes(1, 'little'))
                        payload.extend(message.data[1:2])

                        payload.extend(struct.pack('h', int(Rset)))
                        payload.extend(struct.pack('h', int(Lset)))
                        
                        ##
                        if self.filtering:
                            # print("FILTRO")
                            payload.extend(bytes([2]))
                        else:
                            # print("NO FILTRO")
                            payload.extend(bytes([0]))


                        ack = CANProtocol.create_message(payload)
                        msg = can.Message(
                                            is_fd = True,
                                            bitrate_switch = True,
                                            is_extended_id = False,
                                            arbitration_id = (CANProtocol.MPU_ID() << 6) + CANProtocol.GET_TWIST_ACK(),
                                            data = ack
                                        )

                        vel.linear.x = v
                        vel.linear.y = Rset
                        vel.linear.z = Lset

                        vel.angular.z = w
                        
                        # timer = self.cronometro(v,timer)

                        ## send msg and public vel on topic 
                        self.bus.send(msg)
                        self.vel_pub.publish(vel)

    

    def read_joy_callback(self,msg):

        UpLX = msg.y
        
        # modalità filtering velocity
        if UpLX:
            if (time.time() - self.UpLX_time) > 1:
                self.filtering += 1
                if self.filtering > 1:
                    self.filtering = 0
                else:
                    pass
                self.UpLX_time = time.time()

    def filtering_velocity(self,v):
        if self.filtering:
            if v == 0.0:
                self.v_prec_mod = 0.0
            ## filter velocity
            v = (1 - 0.01)*self.v_prec_mod + 0.01*v
            self.v_prec_mod = v
        
        return v

    ## Utils
    def cronometro(self,v , timer):

        if v < 0:
            if self.set_time:
                self.start_time = time.time()
                self.set_time = False 
            else: 
                timer = time.time() - self.start_time
        if v == 0:
            print("time: " , timer)

        return timer                        
                       
def main():
    print("#########################")
    print("#   ONLY MOVING         #")
    print("#########################")
    ros_to_can = ROStoCANFD('socketcan')

if __name__ == '__main__':
    main()

