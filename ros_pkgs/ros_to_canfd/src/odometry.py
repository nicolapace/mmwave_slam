#!/usr/bin/env python3
import struct
import can
import rospy , ros_numpy
import numpy as np
from canfd_protocol_new import CANProtocol
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import time

class EncodersOdometry():
    def __init__(self, bustype):

        rospy.init_node('encoder_odometry')

        ## init param
        channel = rospy.get_param('~channel')
        bitrate = rospy.get_param('~bitrate')
        data_bitrate = rospy.get_param('~data_bitrate')
        timeout = rospy.get_param('~msg_timeout')

        ## init Bus interface
        bus = can.interface.Bus(channel = channel, bustype = bustype, fd = True, bitrate = bitrate, data_bitrate = data_bitrate)

        odom_pub = rospy.Publisher("odom", Vector3, queue_size=10)
        vel_pub = rospy.Publisher("cmd_vel_encoder", Twist, queue_size=10)

        ## sub to sensors
        # rospy.Subscriber('/diff_robot/ti_mmwave_LX/radar_scan_pcl', PointCloud2, self.cloud_callback_LX, queue_size=1)
        # rospy.Subscriber('/diff_robot/ti_mmwave_FRONT/radar_scan_pcl', PointCloud2, self.cloud_callback_FRONT, queue_size=1)
        # rospy.Subscriber('/diff_robot/ti_mmwave_RX/radar_scan_pcl', PointCloud2, self.cloud_callback_RX, queue_size=1)

        odomTf = tf.TransformListener()
        pose = Vector3()
        vel = Twist()
        frad2deg = 180/np.pi
        rospy.sleep(5)      # sleep fondamentale

        ## extract odom tf from launch file
        transODOM,rotODOM = odomTf.lookupTransform('/diff_robot_base_link','/odom_link', rospy.Time(0))

        # transLX,rotLX = odomTf.lookupTransform('/diff_robot_base_link', '/diff_robot_ti_mmwave_LX',  rospy.Time(0))
        # transFRONT,rotFRONT = odomTf.lookupTransform('/diff_robot_base_link','/diff_robot_ti_mmwave_FRONT', rospy.Time(0))
        # transRX,rotRX = odomTf.lookupTransform('/diff_robot_base_link', '/diff_robot_ti_mmwave_RX',  rospy.Time(0))

        # rotLX = np.array(tf.transformations.quaternion_matrix(rotLX)[:3,:3])
        # transLX = np.array(transLX)
        # rotRX = np.array(tf.transformations.quaternion_matrix(rotRX)[:3,:3])
        # transRX = np.array(transRX)
        # rotFRONT = np.array(tf.transformations.quaternion_matrix(rotFRONT)[:3,:3])
        # transFRONT = np.array(transFRONT)
        tox = transODOM[0]
        toy = transODOM[1]
        ## init Pose 
        self.x = tox
        self.y = 0.0
        self.th = 0.0

        ## init velocity
        self.vx = 0
        self.vy = 0
        self.vth = 0 

        ## init params
        self.lengthWheelBase=0.64
        self.radius= 0.116
        self.rpm_max=3600
        self.coeff_riduz= 43.25

        ## blind spots 
        self.odom_radius = 0.5      # every 80 cm reset dell'odometria

        self.xsub_LX = 0.0
        self.ysub_LX = 0.0 
        self.thsub_LX = 0.0 

        self.xsub_RX = 0.0
        self.ysub_RX = 0.0 
        self.thsub_RX = 0.0

        self.xsub_FRONT = 0.0
        self.ysub_FRONT = 0.0 
        self.thsub_FRONT = 0.0

        self.tracking_point_LX = np.array([[4.0], [4.0] ])
        self.lx_primo = np.array([[4.0], [4.0] ])

        self.tracking_point_RX = np.array([[4.0], [4.0] ])
        self.rx_primo = np.array([[4.0], [4.0] ])

        self.tracking_point_FRONT = np.array([[4.0], [4.0] ])
        self.front_primo = np.array([[4.0], [4.0] ])

        self.last_time = rospy.Time.now() 

        print("ENCODERS ODOMETRY READY")

        ## main loop
        while not rospy.is_shutdown():

            message = bus.recv(1.0)
            if message is not None:

                device_id = CANProtocol.get_device_id(message.arbitration_id)
                message_id = CANProtocol.get_message_id(message.arbitration_id)

                if device_id == CANProtocol.ALL_IN_ONE_ID() and message_id == CANProtocol.DRIVE_CMD_AIO_ACK_ID():
                    if CANProtocol.check_integrity(message.data[0:], CANProtocol.DRIVE_CMD_AIO_ACK_LEN()):

                        ## extract vel_Ang from encoders
                        rpm_l = -struct.unpack('h', message.data[4:6])[0]
                        rpm_r = struct.unpack('h', message.data[8:10])[0]

                        w_r=(2*np.pi*rpm_r/60)/self.coeff_riduz
                        w_l=(2*np.pi*rpm_l/60)/self.coeff_riduz

                        v = self.radius*(w_r+w_l)/2.0
                        w = self.radius*(+w_r-w_l)/self.lengthWheelBase

                        ## extract arr_data from sensors
                        # array_LX = self.arr_data_LX
                        # min_lx_index = np.argmin(np.linalg.norm(array_LX , axis=1))
                        # lx_min = np.dot(rotLX , array_LX[min_lx_index][np.newaxis].T) + transLX[np.newaxis,:].T                 # punto minimo front in xyz
                        # la = np.arctan2(lx_min[1] , lx_min[0])[0]  * frad2deg

                        # array_RX = self.arr_data_RX
                        # min_rx_index = np.argmin(np.linalg.norm(array_RX , axis=1))
                        # rx_min = np.dot(rotRX , array_RX[min_rx_index][np.newaxis].T) + transRX[np.newaxis,:].T                 # punto minimo front in xyz
                        # ra = np.arctan2(rx_min[1] , rx_min[0])[0]  * frad2deg

                        # array_FRONT = self.arr_data_FRONT
                        # min_front_index = np.argmin(np.linalg.norm(array_FRONT , axis=1))
                        # front_min = np.dot(rotFRONT , array_FRONT[min_front_index][np.newaxis].T) + transFRONT[np.newaxis,:].T                 # punto minimo front in xyz
                        # fa = np.arctan2(front_min[1] , front_min[0])[0]  * frad2deg
                        


                        ## ODOMETRY
                        ## restart odom every 0.5 window moving
                        self.restart_odom(tox , toy)

                        ## Calculate Odometry
                        self.odometry_publisher(v,w)
                        
                        ## Center of machine Pose
                        pose.x = self.x - tox      # [m]
                        pose.y = self.y + toy      # [m]
                        pose.z = self.th                    # [rad] ricorda di rimettere in rad -- ora solo per visualizzazione è in gradi

                        ## quando restarto l'odometria dopo aver fatto 0.5m avrò 
                        ## self.x = 0; self.y = 0 e self.th = 0 quindi
                        ## if abs(self.x)<0.03 and abs(self.y)<0.03 and abs(self.th)<0.03: allora agisci sui punti resettando il punto di tracking

                        odom_pub.publish(pose) 

                         
                        # print("odomX: " , pose.x)
                        # print("odomY: " ,pose.y)
                        # print("odomTH: " ,pose.z)
                        # print(" ")


                        ## tracking point 
                        

                        ## trakka punto a prescindere solo se trovi un punto più vicino aggiorna il punto da tracckare
                        ## TrACKING LX
                        # if abs(lx_min[0][0]) < 0.45 and abs(lx_min[1][0]) < 0.7 and ( la < 90-30 or  la > 90+30):
                        #     if lx_min[1][0] < self.lx_primo[1][0]:
                        #         print(" ")
                        #         print("VECCHIO PUNTO ANCORATO")
                        #         print("x: " , self.tracking_point_LX[0][0])
                        #         print("y: " ,self.tracking_point_LX[1][0])
                        #         ## aggiorna il punto da trakkare
                        #         self.tracking_point_LX[0][0] = lx_min[0][0]
                        #         self.tracking_point_LX[1][0] = lx_min[1][0]

                        #         print("NUOVO PUNTO ANCORATO")
                        #         print("x: " , self.tracking_point_LX[0][0])
                        #         print("y: " ,self.tracking_point_LX[1][0])

                        #         ## una volta che si aggiorna il punto vanno resettate le coordinate pose.x e pose.y
                        #         self.xsub_LX = pose.x
                        #         self.ysub_LX = pose.y
                        #         self.thsub_LX = pose.z

                        # ## TrACKING RX
                        # if abs(rx_min[0][0]) < 0.45 and abs(rx_min[1][0]) < 0.7 and ( ra < -90-30 or  ra > -90+30):
                        #     if rx_min[1][0] < self.rx_primo[1][0]:
                        #         print(" ")
                        #         print("VECCHIO PUNTO ANCORATO RX")
                        #         print("x: " , self.tracking_point_RX[0][0])
                        #         print("y: " ,self.tracking_point_RX[1][0])
                        #         ## aggiorna il punto da trakkare
                        #         self.tracking_point_RX[0][0] = rx_min[0][0]
                        #         self.tracking_point_RX[1][0] = rx_min[1][0]

                        #         print("NUOVO PUNTO ANCORATO RX")
                        #         print("x: " , self.tracking_point_RX[0][0])
                        #         print("y: " ,self.tracking_point_RX[1][0])

                        #         ## una volta che si aggiorna il punto vanno resettate le coordinate pose.x e pose.y
                        #         self.xsub_RX = pose.x
                        #         self.ysub_RX = pose.y
                        #         self.thsub_RX = pose.z

                        # ## TrACKING FRONT
                        # if abs(front_min[0][0]) < 0.8 and abs(front_min[1][0]) < 0.38 and ( fa < -30 or  fa > 30):
                        #     if front_min[1][0] < self.front_primo[1][0]:
                        #         print(" ")
                        #         print("VECCHIO PUNTO ANCORATO FRONT")
                        #         print("x: " , self.tracking_point_FRONT[0][0])
                        #         print("y: " ,self.tracking_point_FRONT[1][0])
                        #         ## aggiorna il punto da trakkare
                        #         self.tracking_point_FRONT[0][0] = front_min[0][0]
                        #         self.tracking_point_FRONT[1][0] = front_min[1][0]

                        #         print("NUOVO PUNTO ANCORATO FRONT")
                        #         print("x: " , self.tracking_point_FRONT[0][0])
                        #         print("y: " ,self.tracking_point_FRONT[1][0])

                        #         ## una volta che si aggiorna il punto vanno resettate le coordinate pose.x e pose.y
                        #         self.xsub_FRONT = pose.x
                        #         self.ysub_FRONT = pose.y
                        #         self.thsub_FRONT = pose.z
                                
                        # ## confronta con il punto precedente 
                        # # P in sigma_primo LX
                        # OO_ = np.array([ [pose.x - self.xsub_LX] , [pose.y -  self.ysub_LX] ])
                        # rot_mat = np.array([ [ np.cos(pose.z - self.thsub_LX) , np.sin(pose.z - self.thsub_LX) ] , [ -np.sin(pose.z - self.thsub_LX) , np.cos(pose.z - self.thsub_LX) ] ])
                        # diff = self.tracking_point_LX[:2] - OO_
                        # self.lx_primo = np.dot( rot_mat , diff)

                        # # P in sigma_primo RX
                        # OO_ = np.array([ [pose.x - self.xsub_RX] , [pose.y -  self.ysub_RX] ])
                        # rot_mat = np.array([ [ np.cos(pose.z - self.thsub_RX) , np.sin(pose.z - self.thsub_RX) ] , [ -np.sin(pose.z - self.thsub_RX) , np.cos(pose.z - self.thsub_RX) ] ])
                        # diff = self.tracking_point_RX[:2] - OO_
                        # self.rx_primo = np.dot( rot_mat , diff)

                        # # P in sigma_primo FRONT
                        # OO_ = np.array([ [pose.x - self.xsub_FRONT] , [pose.y -  self.ysub_FRONT] ])
                        # rot_mat = np.array([ [ np.cos(pose.z - self.thsub_FRONT) , np.sin(pose.z - self.thsub_FRONT) ] , [ -np.sin(pose.z - self.thsub_FRONT) , np.cos(pose.z - self.thsub_FRONT) ] ])
                        # diff = self.tracking_point_FRONT[:2] - OO_
                        # self.front_primo = np.dot( rot_mat , diff)

                        
                        

      
    # def cloud_callback_LX(self,msg):
    #     self.arr_data_LX = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    #     self.timer_lx = time.time()

    # def cloud_callback_FRONT(self,msg):
    #     self.arr_data_FRONT = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    #     self.timer_front = time.time()
   
    # def cloud_callback_RX(self,msg):
    #     self.arr_data_RX = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    #     self.timer_rx = time.time()

    # def read_joy_callback(self, msg):
    #     ## reset odom
    #     UpRX = msg.z
        
    #     if UpRX:
    #         if (time.time() - self.UpRX_time) > 1:
    #             self.resetOdom = True
    #             self.UpRX_time = time.time()

    def restart_odom(self , transodom_x , transodom_y):

        r = np.sqrt( (self.x - transodom_x)**2 + (self.y - transodom_y)**2)
        if r > self.odom_radius:
            # print("\nRESET ODOM x: ", self.x)
            # restart odom
            self.x = transodom_x
            self.y = 0.0
            self.th = 0.0

            # self.xsub_LX = 0.0
            # self.ysub_LX = 0.0 
            # self.thsub_LX = 0.0 

            # self.xsub_RX = 0.0
            # self.ysub_RX = 0.0 
            # self.thsub_RX = 0.0 

            # self.xsub_FRONT = 0.0
            # self.ysub_FRONT = 0.0 
            # self.thsub_FRONT = 0.0 
            # print("\nRESET ODOM\n")
        else: 
            pass

    def odometry_publisher(self,vx,w):

        # get time to calculate the dt
        current_time = rospy.Time.now() 
        Ts = (current_time - self.last_time).to_sec()

        last_th = self.th

        ## calculate odometry with exact method
        if w!=0:
            th_k = self.th
            self.th = self.th + w * Ts

            self.bound_angle_th()

            delta_x = ( vx / w ) * (np.sin(self.th) - np.sin(th_k))
            delta_y = -( vx / w ) * (np.cos(self.th) - np.cos(th_k))
        else:
            delta_x = vx * Ts 
            delta_y = 0

        self.x += delta_x
        self.y += delta_y

        # update time
        self.last_time = current_time

    def bound_angle_th(self):

        if self.th < -np.pi:
            self.th += 2*np.pi
        elif self.th > np.pi:
            self.th -= -2*np.pi
        else:
            pass

def main():
    odom = EncodersOdometry('socketcan')

if __name__ == '__main__':
    main()
                           


