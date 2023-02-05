"""
CANFD tools class for a custom protocol
where messages have the following format:
N byte: data
4 byte: crc32 of data field
N byte: one complement of data field
4 byte: crc32 of one complement field
"""

# pylint: disable=c0301
# pylint: disable=c0103

import struct
import binascii

class CANProtocol():
    """
    CAN protocol constants
    """
    #shared attributes
    rolloff = 0
    on_flight_msgs = []


    #devices id
    @staticmethod
    def MASTER_ID():
        """Master board id"""
        return 8
    @staticmethod
    def MPU_ID():
        """MPU id"""
        return 15
    @staticmethod
    def PROGRAMMER_ID():
        """Programmer id"""
        return 20
    @staticmethod
    def LEFT_MOTOR_ID():
        """Left motor id"""
        return 1
    @staticmethod
    def RIGHT_MOTOR_ID():
        """Right motor id"""
        return 4
    @staticmethod
    def ALL_IN_ONE_ID():
        """Right motor id"""
        return 10
    @staticmethod
    def DASHBOARD_ID():
        """Right motor id"""
        return 9


    #define messages id
    @staticmethod
    def DRIVE_CMD_ID():
        """Drive cmd id"""
        return 3
    @staticmethod
    def DRIVE_CMD_ACK_ID():
        """Drive ack id"""
        return 4
    @staticmethod
    def DRIVE_CMD_AIO_ID():
        """Drive cmd aio id"""
        return 29
    @staticmethod
    def DRIVE_CMD_AIO_ACK_ID():
        """Drive ack aio id"""
        return 30
    @staticmethod
    def HEARTBEAT():
        """Heartbeat cmd id"""
        return 31
    @staticmethod
    def HEARTBEAT_ACK():
        """Heartbeat ack id"""
        return 32
    @staticmethod
    def INIT_DIAGNOSTIC():
        """Init diagnostic cmd id"""
        return 33
    @staticmethod
    def INIT_DIAGNOSTIC_ACK():
        """Init diagnostic ack id"""
        return 34
    @staticmethod
    def END_DIAGNOSTIC():
        """End diagnostic cmd id"""
        return 35
    @staticmethod
    def END_DIAGNOSTIC_ACK():
        """End diagnostic ack id"""
        return 36
    @staticmethod
    def SET_SETTINGS():
        """Set setting cmd id"""
        return 37
    @staticmethod
    def SET_SETTINGS_ACK():
        """Set setting ack id"""
        return 38
    @staticmethod
    def SET_IO():
        """Set inout cmd id"""
        return 39
    @staticmethod
    def SET_IO_ACK():
        """Set inout ack id"""
        return 40
    @staticmethod
    def GET_SETTINGS():
        """Get setting cmd id"""
        return 41
    @staticmethod
    def GET_SETTINGS_ACK():
        """Get setting ack id"""
        return 42
    @staticmethod
    def GET_IO():
        """Get inout cmd id"""
        return 43
    @staticmethod
    def GET_IO_ACK():
        """Get inout ack id"""
        return 44
    @staticmethod
    def CONFIRM_SETTINGS():
        """Confirm settings cmd id"""
        return 45
    @staticmethod
    def CONFIRM_SETTINGS_ACK():
        """Confirm settings ack id"""
        return 46
    @staticmethod
    def GET_LOG():
        """Get log cmd id"""
        return 47
    @staticmethod
    def GET_LOG_ACK():
        """Get log ack id"""
        return 48
    @staticmethod
    def GET_MEMORY_INFO():
        """Get memory info cmd id"""
        return 49
    @staticmethod
    def GET_MEMORY_INFO_ACK():
        """Get memory info ack id"""
        return 50
    @staticmethod
    def GET_BAD_BLOCKS():
        """Get bad blocks cmd id"""
        return 51
    @staticmethod
    def GET_BAD_BLOCKS_ACK():
        """Get bad block ack id"""
        return 52
    @staticmethod
    def GET_TWIST_CMD():
        return 61
    @staticmethod
    def GET_TWIST_ACK():
        return 62


    #messages len
    @staticmethod
    def DRIVE_CMD_LEN():
        """Drive cmd len"""
        return 4
    @staticmethod
    def DRIVE_ACK_LEN():
        """Drive cmd len"""
        return 6
    @staticmethod
    def DRIVE_CMD_AIO_LEN():
        """Drive cmd AIO len"""
        return 11
    @staticmethod
    def DRIVE_CMD_AIO_ACK_LEN():
        """Drive cmd AIO ack len"""
        return 15
    @staticmethod
    def HEARTBEAT_LEN():
        """Heartbeat cmd len"""
        return 2
    @staticmethod
    def HEARTBEAT_ACK_LEN():
        """Heartbeat ack len"""
        return 28
    @staticmethod
    def INIT_DIAGNOSTIC_LEN():
        """init diagnostic cmd len"""
        return 6
    @staticmethod
    def INIT_DIAGNOSTIC_ACK_LEN():
        """Init diagnostic ack len"""
        return 12
    @staticmethod
    def END_DIAGNOSTIC_LEN():
        """End diagnostic cmd len"""
        return 2
    @staticmethod
    def END_DIAGNOSTIC_ACK_LEN():
        """End diagnostic ack len"""
        return 2
    @staticmethod
    def SET_SETTINGS_LEN():
        """Set seting cmd len"""
        return 28
    @staticmethod
    def SET_SETTINGS_ACK_LEN():
        """Set setting ack len"""
        return 28
    @staticmethod
    def SET_IO_LEN():
        """Set inout cmd len"""
        return 28
    @staticmethod
    def SET_IO_ACK_LEN():
        """Set setting ack len"""
        return 28
    @staticmethod
    def GET_SETTINGS_LEN():
        """Get setting cmd len"""
        return 28
    @staticmethod
    def GET_SETTINGS_ACK_LEN():
        """Get setting ack len"""
        return 28
    @staticmethod
    def CONFIRM_SETTINGS_LEN():
        """Confirm settings cmd len"""
        return 2
    @staticmethod
    def CONFIRM_SETTINGS_ACK_LEN():
        """Confirm settings ack len"""
        return 2
    @staticmethod
    def GET_LOG_LEN():
        """Get log cmd len"""
        return 9
    @staticmethod
    def GET_LOG_ACK_LEN():
        """Get log ack len"""
        return 28
    @staticmethod
    def GET_MEMORY_INFO_LEN():
        """Get memory info cmd len"""
        return 2
    @staticmethod
    def GET_MEMORY_INFO_ACK_LEN():
        """Get memory info ack len"""
        return 15
    @staticmethod
    def GET_BAD_BLOCKS_LEN():
        """Get bad block cmd len"""
        return 15
    @staticmethod
    def GET_BAD_BLOCKS_ACK_LEN():
        """Get bad block ack len"""
        return 28
    @staticmethod
    def GET_TWIST_CMD_LEN():
        return 6
    @staticmethod
    def GET_TWIST_ACK_LEN():
        return 6

        
    #other const
    @staticmethod
    def ACK_TIMEOUT():
        """Ack reception timeout"""
        return 5000
    @staticmethod
    def HEARTBEAT_INTERVAL():
        """Heartbeat interval"""
        return 1000
    @staticmethod
    def MAX_RETRANSMISSION():
        """Max retransmission number before error"""
        return 10

    @staticmethod
    def get_rolloff():
        """
        Get rolloff
        """
        if CANProtocol.rolloff < 255:
            CANProtocol.rolloff += 1
        else:
            CANProtocol.rolloff = 0
        return CANProtocol.rolloff

    @staticmethod
    def insert_in_queue(msg_id, rolloff):
        """
        Insert message in queue
        """
        CANProtocol.on_flight_msgs.append([msg_id + 1, rolloff])

    @staticmethod
    def remove_from_queue(msg_id, rolloff):
        """
        Remove message from queue
        """
        for i, msg in enumerate(CANProtocol.on_flight_msgs):
            if msg_id == msg[0] and rolloff == msg[1]:
                CANProtocol.on_flight_msgs.pop(i)
                return True
        return False

    @staticmethod
    def remove_all():
        """
        Remove all message from queue
        """
        CANProtocol.on_flight_msgs.clear()

    @staticmethod
    def check_pending_messages():
        """
        Check if message queue is not full
        """
        for msg in CANProtocol.on_flight_msgs:
            if msg[0] != CANProtocol.HEARTBEAT():
                return True

    @staticmethod
    def create_heartbeat_message():
        """
        Create heartbeat message
        """
        header = [CANProtocol.HEARTBEAT()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_init_diagnostic_message(hour, day ,month, year):
        """
        Create init diagnostic message
        """
        header = [CANProtocol.INIT_DIAGNOSTIC()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        data += hour.to_bytes(1, 'little')
        data += day.to_bytes(1, 'little')
        data += month.to_bytes(1, 'little')
        data += year.to_bytes(1, 'little')
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_end_diagnostic_message():
        """
        Create end diagnostic message
        """
        header = [CANProtocol.END_DIAGNOSTIC()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_set_settings_message(settings):
        """
        Create set setting message
        """
        header = [CANProtocol.SET_SETTINGS()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        data += settings
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_set_inout_message(settings):
        """
        Create set inout messsage
        """
        header = [CANProtocol.SET_IO()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        data += settings
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_get_settings_message(settings):
        """
        Create get setting message
        """
        header = [CANProtocol.GET_SETTINGS()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        data += settings
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_get_inout_message(settings):
        """
        Create get inout message
        """
        header = [CANProtocol.GET_IO()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        data += settings
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_confirm_settings_message():
        """
        Create confirm settings message
        """
        header = [CANProtocol.CONFIRM_SETTINGS()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_get_log_message(start_block, start_page, start_byte, end_byte):
        """
        Create get log message
        """
        header = [CANProtocol.GET_LOG()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        data += start_block.to_bytes(2, 'little')
        data += start_page.to_bytes(1, 'little')
        data += start_byte.to_bytes(2, 'little')
        data += end_byte.to_bytes(2, 'little')
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_get_memory_info_message():
        """
        Create get memory info message
        """
        header = [CANProtocol.GET_MEMORY_INFO()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_get_badblocks_message(index):
        """
        Create get bad blocks message
        """
        header = [CANProtocol.GET_BAD_BLOCKS()]
        data = bytearray()
        data += CANProtocol.MASTER_ID().to_bytes(1, 'little')
        data += CANProtocol.get_rolloff().to_bytes(1, 'little')
        data += index
        payload = CANProtocol.create_message(data)
        return bytearray(header) + payload

    @staticmethod
    def create_message(data):
        """
        create CAN message with data, crc32, complement and crc32

        data: data to send as list

        return complete message as bytearray
        """
        message = bytearray()
        message.extend(data)
        crc = binascii.crc32(bytearray(data))
        message.extend(struct.pack('I', crc))
        complement = []
        for dat in data:
            complement.append(255 - dat)
        message.extend(complement)
        crc = binascii.crc32(bytearray(complement))
        message.extend(struct.pack('I', crc))
        return message

    @staticmethod
    def check_integrity(frame, msg_len):
        """
        check frame integrity

        frame: frame received

        msg_len: data field length

        return true if the frame has no error
        """
        data = frame[:msg_len]
        if CANProtocol.check_crc32(data, frame[msg_len:msg_len+4]):
            if CANProtocol.check_one_complement(data, frame[msg_len+4:msg_len*2+4]):
                if CANProtocol.check_crc32(frame[msg_len+4:msg_len*2+4], frame[msg_len*2+4: (msg_len+4)*2]):
                    return True
        return False

    @staticmethod
    def check_crc32(data, expected):
        """
        calculate crc32 of a given data and compare it with an expected value

        data: data fields of the frame

        expected: crc fields of the frame

        return true if calculated crc32 is equal to expected crc32
        """
        exepcted_crc = 0
        crc = 0

        for i, expe in enumerate(expected):
            exepcted_crc += (expe << 8*i)
        crc = binascii.crc32(bytearray(data))
        if crc == exepcted_crc:
            return True
        return False

    @staticmethod
    def check_one_complement(data, expected):
        """
        calculate one complement of a given data and compare it with an expected value

        data: data fields of the frame

        expected: one complement fields of the frame

        return true if calculated one complement is equal to expected one complement
        """
        for i, expe in enumerate(expected):
            if (255 - data[i]) != expe:
                return False
        return True

    @staticmethod
    def get_data(frame, msg_len):
        """
        return data field of the frame as bytearray

        frame: received frame as bytearray

        msg_len: data field length
        """
        message = bytearray()
        message.extend(frame[:msg_len])
        return message

    @staticmethod
    def get_message_id(can_id):
        """
        Return msg id

        id: CAN message id
        """
        return can_id & 0x3F

    @staticmethod
    def get_device_id(can_id):
        """
        Retun device id

        id: CAN message id
        """
        return can_id >> 6
