#!/usr/bin/env python3
import struct
import can
import rospy , ros_numpy
import numpy as np
from math import sin, cos, pi

'''
comandi:

'''

from canfd_protocol_odom import CANProtocol
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

method="exact"
    
def main():

    print("#########################")
    print("# EncoderOdometry  node #")
    print("#########################")

    ## init param
    channel = rospy.get_param('~channel')
    bitrate = rospy.get_param('~bitrate')
    data_bitrate = rospy.get_param('~data_bitrate')
    timeout = rospy.get_param('~msg_timeout')
    bustype='socketcan'
    print("Channel:\t", channel)
    print("Bitrate:\t", bitrate)
    print("Data bitrate:\t", data_bitrate)
    print("Msg timeout:\t", timeout)

 

    ## init Bus interface
    bus = can.interface.Bus(channel = channel, bustype = bustype, fd = True, bitrate = bitrate, data_bitrate = data_bitrate)

    ## main loop
    while not rospy.is_shutdown():

        message = bus.recv(1.0)
        if message is not None:

            device_id = CANProtocol.get_device_id(message.arbitration_id)
            message_id = CANProtocol.get_message_id(message.arbitration_id)

            #print("device_id: "+ str(device_id) + " message_id: "  + str(message_id))

            if device_id == CANProtocol.ALL_IN_ONE_ID() and message_id == CANProtocol.DRIVE_CMD_AIO_ACK_ID():
                if CANProtocol.check_integrity(message.data[0:], CANProtocol.DRIVE_CMD_AIO_ACK_LEN()):
                    ## extract vel_Ang
                    rpm_l = -struct.unpack('h', message.data[4:6])[0]
                    rpm_r = struct.unpack('h', message.data[8:10])[0]
                    #print("vel_ang left wheel hex: "+hex(message.data[3])+ " " +hex(message.data[4]))
                    #print("vel_ang left wheel hex struct: "+hex(vel_ang_left))
                    print("rpm_l: "+str(int(rpm_l)))
                    print("rpm_r: "+str(int(rpm_r)))

                    

                    lengthWheelBase=0.64
                    radius= 0.116

                    rpm_max=3600
                    coeff_riduz= 43.25

                    w_r=(2*pi*rpm_r/60)/coeff_riduz
                    w_l=(2*pi*rpm_l/60)/coeff_riduz

                    vx = radius*(w_r+w_l)/2.0
                    vth = radius*(+w_r-w_l)/lengthWheelBase # (right_speed - left_speed)/lengthWheelBase ;
                    odometry_publisher(vx,vth)


def odometry_publisher(vx,w):

    global last_time
    # get time to calculate the dt
    current_time = rospy.Time.now() 
    Ts = (current_time - last_time).to_sec()

    global x
    global y
    global th

    # compute odometry given the velocities of the robot
    if method=="euler":
        #first order numerical integration (Ts = dt = t_k+1 - t_k )
        delta_th = w * Ts
        delta_x = vx * Ts * cos(th)  #(vx * cos(th) - vy * sin(th)) * Ts
        delta_y = vx * Ts * sin(th)  #(vx * sin(th) + vy * cos(th)) * Ts
    elif method=="runge-kutta":
        # th + w*Ts/2 is the average orentation of the robot in [t_k, t_k+1)
        delta_th = w * Ts
        delta_x = vx * Ts * cos(th + w*Ts/2) 
        delta_y = vx * Ts * sin(th + w*Ts/2)
    elif method=="exact":
        # exact integration 
        if w!=0:
            delta_th = w * Ts
            th_k = th
            th_k1 = th + delta_th
            delta_x = ( vx / w ) * (sin(th_k1) - sin(th_k))
            delta_y = -( vx / w ) * (cos(th_k1) - cos(th_k))
        else:
            delta_th = 0
            delta_x = vx * Ts 
            delta_y = 0
       

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    print("angolo:  "+str(360*th/(2*pi)))
    print("x: "+str(x))

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform((x, y, 0.),odom_quat,current_time,"base_link","odom")

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    # update time
    last_time = current_time


if __name__ == '__main__':

    rospy.init_node('encoder_odometry')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0
    vy = 0
    vth = 0 

    last_time = rospy.Time.now()
    current_time = rospy.Time.now()

    main()
                           


