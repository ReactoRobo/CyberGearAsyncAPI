"""
I will arise and go now, and go to Innisfree,

And a small cabin build there, of clay and wattles made
"""

import struct
import logging
import enum
import math
import serial
import time
import asyncio
import aioserial
from construct import BitStruct, BitsInteger

class CANMotorAsyncController:
    PARAM_TABLE = {
        "motorOverTemp": {"feature_code": 0x200D, "type": "int16"},
        "overTempTime": {"feature_code": 0x200E, "type": "int32"},
        "limit_torque": {"feature_code": 0x2007, "type": "float"},
        "cur_kp": {"feature_code": 0x2012, "type": "float"},
        "cur_ki": {"feature_code": 0x2013, "type": "float"},
        "spd_kp": {"feature_code": 0x2014, "type": "float"},
        "spd_ki": {"feature_code": 0x2015, "type": "float"},
        "loc_kp": {"feature_code": 0x2016, "type": "float"},
        "spd_filt_gain": {"feature_code": 0x2017, "type": "float"},
        "limit_spd": {"feature_code": 0x2018, "type": "float"},
        "limit_cur": {"feature_code": 0x2019, "type": "float"},
    }

    PARAMETERS = {
        "run_mode": {"index": 0x7005, "format": "u8"},
        "iq_ref": {"index": 0x7006, "format": "f"},
        "spd_ref": {"index": 0x700A, "format": "f"},
        "limit_torque": {"index": 0x700B, "format": "f"},
        "cur_kp": {"index": 0x7010, "format": "f"},
        "cur_ki": {"index": 0x7011, "format": "f"},
        "cur_filt_gain": {"index": 0x7014, "format": "f"},
        "loc_ref": {"index": 0x7016, "format": "f"},
        "limit_spd": {"index": 0x7017, "format": "f"},
        "limit_cur": {"index": 0x7018, "format": "f"},
    }


    TWO_BYTES_BITS = 16

    def __init__(self, uart: aioserial.AioSerial, motor_id=127, main_can_id=254):
        """
        Initial CAN Motor Controller

        Params:
        bus: CAN bus
        motor_id: Motor CAN ID
        main_can_id: Master CAN ID
        """
        self.uart = uart
        self.MOTOR_ID = motor_id
        self.MAIN_CAN_ID = main_can_id
        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -30.0
        self.V_MAX = 30.0
        self.T_MIN = -12.0
        self.T_MAX = 12.0
        self.I_MAX = 27
        self.KP_MIN, self.KP_MAX = 0.0, 500.0  # 0.0 ~ 500.0
        self.KD_MIN, self.KD_MAX = 0.0, 5.0  # 0.0 ~ 5.0
        self.READ = 0
        self.READ_FLAG = 0
        self.ERROR_FLAG = ''
        self.TORQUE_CONSTANT = self.T_MAX / self.I_MAX
        self.Motor_Status = [0]*3
        self.IndexValue = 0
        self.FEEDBACK_STRUCT = BitStruct(
            "com_mode" / BitsInteger(5),
            "motor_mode" / BitsInteger(2),
            "error_code" / BitsInteger(6),
            "motor_canid" / BitsInteger(8),
            "master_canid" / BitsInteger(8),
            "filled_zero" / BitsInteger(3),
        )

        self.WRITE_STRUCT = BitStruct(
            "com_mode" / BitsInteger(5),
            "blank_area" / BitsInteger(8),
            "master_canid" / BitsInteger(8),
            "motor_canid" / BitsInteger(8),
            "filled_zero" / BitsInteger(3),
        )
        self.Serial_Read_Handle_Task = asyncio.create_task(self.async_serial_data_handle())


    # Communication Type
    class CmdModes:
        GET_DEVICE_ID = 0
        MOTOR_CONTROL = 1
        MOTOR_FEEDBACK = 2
        MOTOR_ENABLE = 3
        MOTOR_STOP = 4
        SET_MECHANICAL_ZERO = 6
        SET_MOTOR_CAN_ID = 7
        PARAM_TABLE_WRITE = 8
        SINGLE_PARAM_READ = 17
        SINGLE_PARAM_WRITE = 18
        FAULT_FEEDBACK = 21
    
    # Control Mode
    class RunModes(enum.Enum):
        CONTROL_MODE = 0 # Dynamic Mode
        POSITION_MODE = 1 # Position Mode
        SPEED_MODE = 2 # Speed Mode
        CURRENT_MODE = 3 # Current Mode

    


    def _float_to_uint(self, x, x_min, x_max, bits):
        """
        float -> uint

        Params:
        x: float
        x_min: Minimal float
        x_max: Maximal flot
        bits: uint bits

        Return:
        uint
        """
        span = x_max - x_min
        offset = x_min
        x = max(min(x, x_max), x_min)  # Clamp x to the range [x_min, x_max]
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def _uint_to_float(self, x, x_min, x_max, bits):
        """
        uint -> float

        Params:
        x: uint
        x_min: Minimal float
        x_max: Maximal flot
        bits: uint bits

        Return:
        float
        """
        span = (1 << bits) - 1
        offset = x_max - x_min
        x = max(min(x, span), 0)  # Clamp x to the range [0, span]
        return offset * x / span + x_min

    def _linear_mapping(
        self, value, value_min, value_max, target_min=0, target_max=65535
    ):
        """
        Linear Mapping

        Params:
        value: input value
        value_min: input lower border
        value_max: input upper border
        target_min: output lower border
        target_max: output upper border

        Return:
        映射后的值。
        """
        return value_min + value / (target_max - target_min) * (value_max - value_min)


    def can_to_uart(self, data=[], rtr=0):
        udata = [0x41, 0x54, 0x0, 0x0, 0x0, 0x0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x0a]
        if len(data) == 13 and data[4] == 0x08:
            for i in range(13):
                udata[2 + i] = data[i]

            return udata
        
        else:
            return []


    # USB转CAN模块包模式：串行帧->CAN报文
    def uart_to_can(self, data=[]):
        cdata = [0x0, 0x0, 0x0, 0x0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        if len(data) == 17 and data[6] == 0x08:
            for i in range(13):
                cdata[i] = data[i + 2]
            return cdata

    def organize_can_message(self, cmd_mode=0, cmd_data=[], data=[], data_num = 0x08, rtr=0):
        cdata = [0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] if data_num==0x08 else [0x00, 0x00, 0x00, 0x00, 0x00]
        cdata[0] = (cmd_mode << 3) | (cmd_data[1] >> 5)
        cdata[1] = (cmd_data[1] << 3) | (cmd_data[0] >> 5)
        cdata[2] = ((cmd_data[0] << 3) | (self.MOTOR_ID >>5)) & 0xff    # master_ID
        cdata[3] = (self.MOTOR_ID << 3) | 0x04
        cdata[4] = data_num
        for i in range(data_num):
            cdata[5 + i] = data[i]
        return cdata

    def motor_status_parser(self, data: bytearray):
        pos_value = int.from_bytes(data[:2], 'big')
        pos = self._linear_mapping(pos_value, -4*3.14, 4*3.14)
        vel_value = int.from_bytes(data[2:4], 'big')
        vel = self._linear_mapping(vel_value, -30, 30)
        tau_value = int.from_bytes(data[4:6], 'big')
        tau = self._linear_mapping(tau_value, -12, 12)
        temp_value = int.from_bytes(data[6:8], 'big')
        temp = temp_value/10
        return pos, vel, tau, temp


    def cancel_serial_read_handle(self):
        self.Serial_Read_Handle_Task.cancel()
    
    def open_serial_read_handle(self):
        self.Serial_Read_Handle_Task = asyncio.create_task(self.async_serial_data_handle())

    async def async_serial_data_handle(self):
        try:
            while True:
                self.uart.reset_input_buffer()
                data = await self.uart.readline_async()
                if len(data) == 17:
                    data_parser = self.FEEDBACK_STRUCT.parse(data[2:6])
                    if data_parser.com_mode == 2:
                        self.Motor_Status = list(self.motor_status_parser(data[7:15]))
                    elif data_parser.com_mode == 17:
                        self.IndexValue = self.format_data(data=data[11:15], format='f', type="decode")[0]
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(e)
        finally:
            print("clean up")

    async def async_write_data(self, data=[]):
        try:
            await self.uart.write_async(data)
            
        except Exception as e:
            self.uart.close()
            self.uart.open()
            await self.uart.write_async(data)

    async def async_send_command(self, cmd_mode=0, cmd_data=[], data=[], data_num = 0x08, rtr=0):
        cdata = self.organize_can_message(cmd_mode, cmd_data, data, data_num, rtr)
        data = self.can_to_uart(data=cdata, rtr=rtr)
        await self.async_write_data(data=data)

    async def async_write_property(self, index=0, data_type='f', value=0):
        """

        Write Motor Params

        Params:
            id_num: Motor CAN ID
            index: Param Index
            data_type: data type:'f':float,'u16':uint16,'s16':int16,'u32':uint32,'s32':int32,'u8':uint8,'s8':'int8'
            value: data value


        Returns:
            None

        Raises:
            None

        """
        master_id = 0x0
        cmd_data = [0] * 2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0] * 8
        tx_data[0] = index & 0xFF
        tx_data[1] = (index >> 8) & 0xFF
        value = value
        cmd_mode = 18
        if index < 0x7000:
            cmd_mode = 8
            type_list = ['u8', 's8', 'u16', 's16', 'u32', 's32', 'f']
            tx_data[2] = type_list.index(data_type)
        tx_data[4:] = self.format_data(data=[value], format=data_type, type="encode")
        await self.async_send_command(cmd_mode=cmd_mode, cmd_data=cmd_data, data=tx_data, data_num=0x08, rtr=0)

        if cmd_mode == 8: 
            await self.async_disable()
            cmd_data[1] = 0x02
            tx_data = [0] * 8
            await self.async_send_commandsend_command(cmd_mode=cmd_mode, cmd_data=cmd_data, data=tx_data, data_num=0x08, rtr=0)
            time.sleep(0.1) 
    
    async def async_read_property(self, index=0, data_type='f'):
        """
        Read Motor Params

        Params:
            id_num: Motor CAN ID
            index: Param Index
            data_type: data type:'f':float,'u16':uint16,'s16':int16,'u32':uint32,'s32':int32,'u8':uint8,'s8':'int8'

        Returns:
            value: data value

        Raises:
            None

        """
        master_id = 253
        cmd_data = [0] * 2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0] * 8
        tx_data[0] = index & 0xFF
        tx_data[1] = (index >> 8) & 0xFF
        cmd_mode = 17
        if index < 0x7000:
            cmd_mode = 9
            type_list = ['u8', 's8', 'u16', 's16', 'u32', 's32', 'f']
            tx_data[2] = type_list.index(data_type)
        
        self.READ = 1
        await self.async_send_command(cmd_mode=cmd_mode, cmd_data=cmd_data, data=tx_data, data_num=0x08, rtr=0)

        
    async def async_set_mode(self, mode=0):
        """
        Set Motor Mode

        Args:
            id_num: Motor CAN ID
            mode: mode index

        Returns:
            None

        Raises:
            None

        """
        await self.async_write_property(index=0x7005, value=mode, data_type='u8')


    async def async_enable(self):
        """
        Enable Motor
        """
        cmd_type = 0x03
        master_id = 253
        cmd_data_2 = [0x00] * 2
        cmd_data_2[0] = master_id & 0xFF
        tx_data = [0x00]*8
        await self.async_send_command(cmd_mode=cmd_type, cmd_data=cmd_data_2, data=tx_data, rtr=0)  
        

    async def async_disable(self):
        """
        Disable Motor
        """
        master_id = 0x0
        cmd_data = [0] * 2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0] * 8
        for i in range(8):
            tx_data[i] = 0x00
        await self.async_send_command(cmd_mode=4, cmd_data=cmd_data, data=tx_data, data_num=0x08, rtr=0)  


    async def async_set_0_pos(self):
        """
        Set Motor Zero Point
        """
        await self.async_disable()
        master_id = 0x0
        cmd_data = [0] * 2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0] * 8
        tx_data[0] = 0x01
        await self.async_send_command(cmd_mode=6, cmd_data=cmd_data, data=tx_data, data_num=0x08, rtr=0)  


    def format_data(self, data=[], format="f f", type="decode"):
        """
        Decode or Encode data

        Params
            data: data list
            format: data type
            type: "encode" or "decode"

        Return:
            transformed data
        """
        format_list = format.split()
        rdata = []
        if type == "decode":
            p = 0
            for f in format_list:
                s_f = []
                if f == "f":
                    s_f = [4, "f"]
                elif f == "u16":
                    s_f = [2, "H"]
                elif f == "s16":
                    s_f = [2, "h"]
                elif f == "u32":
                    s_f = [4, "I"]
                elif f == "s32":
                    s_f = [4, "i"]
                elif f == "u8":
                    s_f = [1, "B"]
                elif f == "s8":
                    s_f = [1, "b"]
                ba = bytearray()
                if len(s_f) == 2:
                    for i in range(s_f[0]):
                        ba.append(data[p])
                        p = p + 1
                    rdata.append(struct.unpack(s_f[1], ba)[0])
                else:
                    logging.info("unknown format in format_data(): " + f)
                    return []
            return rdata
        elif type == "encode" and len(format_list) == len(data):
            for i in range(len(format_list)):
                f = format_list[i]
                s_f = []
                if f == "f":
                    s_f = [4, "f"]
                elif f == "u16":
                    s_f = [2, "H"]
                elif f == "s16":
                    s_f = [2, "h"]
                elif f == "u32":
                    s_f = [4, "I"]
                elif f == "s32":
                    s_f = [4, "i"]
                elif f == "u8":
                    s_f = [1, "B"]
                elif f == "s8":
                    s_f = [1, "b"]
                if f != "f":
                    data[i] = int(data[i])
                if len(s_f) == 2:
                    bs = struct.pack(s_f[1], data[i])
                    for j in range(s_f[0]):
                        rdata.append(bs[j])
                else:
                    logging.info("unkown format in format_data(): " + f)
                    return []
            if len(rdata) < 4:
                for i in range(4 - len(rdata)):
                    rdata.append(0x00)
            return rdata














