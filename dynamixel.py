#!/usr/bin/env python
import os
import glob
import ctypes
import dynamixel_functions as dynamixel

                            # Dynamixel moving status threshold

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

def get_available_ports():
    return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyCOM*")
class Dxl:
    def __init__(self, DeviceName):
        self.portHandler = dynamixel.portHandler(DeviceName)
        self.packetHandler = dynamixel.packetHandler()
        self.dxl_comm_result = COMM_TX_FAIL
        self.dxl_error = 0
        self.dxl_addparam_result = 0
        self.LEN_MX_GOAL_POSITION        = 2
        self.LEN_MX_PRESENT_POSITION     = 2
        self.ADDR_MX_TORQUE_ENABLE       = 24                            # Control table address is different in Dynamixel model
        self.ADDR_MX_GOAL_POSITION       = 30
        self.ADDR_MX_PRESENT_POSITION    = 36
        self.ADDR_MX_SPEED               = 32
        self.PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel
        self.groupSync = dynamixel.groupSyncWrite(self.portHandler, self.PROTOCOL_VERSION, self.ADDR_MX_GOAL_POSITION, self.LEN_MX_GOAL_POSITION)

# Protocol version

# Default setting
        self.BAUDRATE                    = 1000000

        self.TORQUE_ENABLE               = 1                             # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                             # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 100                           # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE  = 4000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MOVING_STATUS_THRESHOLD = 10
        if(dynamixel.openPort(self.portHandler)):
            print "Port open done"
        else:
            print("Failed to open port")
            quit()
        if(dynamixel.setBaudRate(self.portHandler, self.BAUDRATE)):
            print "Change baudrate succeeded"
        else:
            print "Cannot add baudrate"
            quit()

    def scan(self, ran = 254):
        _ids = []
        for i in range(ran):
            dynamixel.ping(self.portHandler, self.PROTOCOL_VERSION, i)
            dxl_comm_result = dynamixel.getLastTxRxResult(self.portHandler, self.PROTOCOL_VERSION)
            dxl_error = dynamixel.getLastRxPacketError(self.portHandler, self.PROTOCOL_VERSION)
            if(dxl_comm_result != COMM_SUCCESS):
                pass
            elif(dxl_error != 0):
                pass
            else:
                _ids.append(i)

        return _ids

    def _write(self, DXL_ID, angle):
        dynamixel.write2ByteTxRx(self.portHandler, self.PROTOCOL_VERSION, DXL_ID, self.ADDR_MX_GOAL_POSITION, 2048 + int(angle / 0.088))
        dxl_comm_result= dynamixel.getLastTxRxResult(self.portHandler, self.PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.portHandler, self.PROTOCOL_VERSION)
        if(dxl_comm_result != COMM_SUCCESS):
            print(dynamixel.getTxRxResult(self.PROTOCOL_VERSION, self.dxl_comm_result))
        elif(dxl_error != 0):
            print(dynamixel.getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
    
    def _read(self, DXL_ID):
        dxl_present_position = dynamixel.read2ByteTxRx(self.portHandler, self.PROTOCOL_VERSION, DXL_ID, self.ADDR_MX_PRESENT_POSITION)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.portHandler, self.PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.portHandler, self.PROTOCOL_VERSION)
        if(dxl_comm_result != COMM_SUCCESS):
            print(dynamixel.getTxRxResult(self.PROTOCOL_VERSION, self.dxl_comm_result))
        elif(dxl_error != 0):
            print(dynamixel.getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
        return (dxl_present_position - 2048) * 0.088
    def set_goal_position(self, ids):
        # for i, angle in ids.iteritems():
            # self._write(i,angle)
        for i, angle in ids.iteritems():
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(self.groupSync, i, 2048 + int(angle / 0.088), self.LEN_MX_GOAL_POSITION)).value
        dynamixel.groupSyncWriteTxPacket(self.groupSync)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.portHandler, self.PROTOCOL_VERSION)
        if(dxl_comm_result != COMM_SUCCESS):
            print(dynamixel.getTxRxResult(self.PROTOCOL_VERSION, self.dxl_comm_result))
        dynamixel.groupSyncWriteClearParam(self.groupSync)


    def get_present_position(self, ids):
        present_pos = {}
        for i in ids:
            present_pos[i] = self._read(i)
        return present_pos
    def _set_moving_speed(self, DXL_ID, speed):
        dynamixel.write2ByteTxRx(self.portHandler, self.PROTOCOL_VERSION, DXL_ID, self.ADDR_MX_SPEED, int(speed * 1024 / 1000))
        dxl_comm_result = dynamixel.getLastTxRxResult(self.portHandler, self.PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.portHandler, self.PROTOCOL_VERSION)
        if(dxl_comm_result != COMM_SUCCESS):
            print(dynamixel.getTxRxResult(self.PROTOCOL_VERSION, self.dxl_comm_result))
        elif(dxl_error != 0):
            print(dynamixel.getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
    def set_moving_speed(self, ids):
        for i,val in ids.iteritems():
            self._set_moving_speed(i,val)

    def _enable_torque(self, DXL_ID):
        dynamixel.write2ByteTxRx(self.portHandler, self.PROTOCOL_VERSION, DXL_ID, self.ADDR_MX_TORQUE_ENABLE, 1)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.portHandler, self.PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.portHandler, self.PROTOCOL_VERSION)
        if(dxl_comm_result != COMM_SUCCESS):
            print(dynamixel.getTxRxResult(self.PROTOCOL_VERSION, self.dxl_comm_result))
        elif(dxl_error != 0):
            print(dynamixel.getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
    
    def enable_torque(self, ids):
        for i in ids:
            self._enable_torque(i)

    def _disable_torque(self, DXL_ID):
        dynamixel.write2ByteTxRx(self.portHandler, self.PROTOCOL_VERSION, DXL_ID, self.ADDR_MX_TORQUE_ENABLE, 0)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.portHandler, self.PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.portHandler, self.PROTOCOL_VERSION)
        if(dxl_comm_result != COMM_SUCCESS):
            print(dynamixel.getTxRxResult(self.PROTOCOL_VERSION, self.dxl_comm_result))
        elif(dxl_error != 0):
            print(dynamixel.getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
    def disable_torque(self, ids):
        for i in ids:
            self._enable_torque(i)
# ports = get_available_ports()

# d = Dxl(ports[0])
# ids = d.scan(30)
# for i in ids:
    # d._write(i, 0)
