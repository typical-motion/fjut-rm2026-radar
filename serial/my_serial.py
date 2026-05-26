import threading
from serial import Serial
import time
import struct

double_vulnerability_chance = -1    # 双倍易伤机会数
opponent_double_vulnerability = -1  # 是否正在触发双倍易伤
chances_flag = 1  # 双倍易伤触发标志位，需要从1递增，每小局比赛会重置，所以每局比赛要重启程序
progress_list = [-1, -1, -1, -1, -1, -1]  # 标记进度列表

# 标记进度
mark_value = {
    "B1": 0,
    "B2": 0,
    "B7": 0,
    "R1": 0,
    "R2": 0,
    "R7": 0
}

# 机器人名字对应ID
mapping_table = {
    "R1": 1,
    "R2": 2,
    "R3": 3,
    "R4": 4,
    "R5": 5,
    "R6": 6,
    "R7": 7,
    "B1": 101,
    "B2": 102,
    "B3": 103,
    "B4": 104,
    "B5": 105,
    "B6": 106,
    "B7": 107
}

# CRC校验
# CRC8校验表：G(x)=x8+x5+x4+1
CRC8_INIT = 0xff
CRC8_TAB = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]

# CRC16校验表
CRC_INIT = 0xffff
wCRC_Table = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]

# CRC8校验
def Get_CRC8_Check_Sum(pchMessage, dwLength):
        ucCRC8 = CRC8_INIT
        for ch in pchMessage[:dwLength]:
            ucIndex = ucCRC8 ^ ch
            ucCRC8 = CRC8_TAB[ucIndex]
        return ucCRC8
    
# CRC16校验
def Get_CRC16_Check_Sum(pchMessage, dwLength):
    wCRC = CRC_INIT
    for ch in pchMessage[:dwLength]:
        wCRC = ((wCRC >> 8) & 0xFF) ^ wCRC_Table[(wCRC ^ ch) & 0xFF]
    return wCRC

# 解析单个数据包
# 帧头frame_header格式：SOF(1-byte) + data_length(2-byte) + seq(1-byte) + CRC8(1-byte)
# 整个协议格式: frame_header(5-byte) + cmd_id(2-byte) + data(n-byte) + frame_tail(2-byte, CRC16)
# data是需要处理的数据包；cmd_id是相应的序列号; info是指要不要输出日志，一般选True\False
def receive_packet(data, cmd_id, info=False):
    # 先找到帧头并提取出来, 如果没有SOF（即index为-1），就返回空值
    sof_index = data.find(b'\xA5')      # SOF = 0xA5
    if sof_index == -1:
        if info:
            print('找不到SOF')
        return None
    frame_header = data[sof_index : sof_index+5]    #提取帧头，frame_header(5-byte)
    
    # 提取帧头的第二项，data_length(整个帧头：第0个是SOF，第1个和第2个是data_length, 第3个是seq， 第4个是CRC8校验)
    data_length_bytes = frame_header[1:3]
    data_length = int.from_bytes(data_length_bytes, byteorder='little')
    if len(data) < 5+data_length+2:     # 帧头5-byte + 数据长度 + 帧尾2-byte
        if info:
            print('数据量不足')
        return None
    
    # 提取帧头第三项，seq（每次发送序列号都会自增1，到256时又会重新变回0）
    seq = frame_header[3]
    
    # CRC8校验
    crc8 = Get_CRC8_Check_Sum(frame_header[:-1], 5-1)
    if crc8 != frame_header[-1]:
        if info:
            print('CRC8校验失败')
        return None
    
    # 提取命令码,位置在帧头之后的第一个，于是从sof位置+帧头5-byte开始，此外cmd_id有2-byte，所以还要+2
    # 用来检查收到的cmd_id_bytes和期望的cmd_id是不是匹配
    cmd_id_bytes = data[sof_index+5 : sof_index+5+2]
    cmd_id = bytes([cmd_id[1], cmd_id[0]])
    if cmd_id_bytes != cmd_id:
        if info:
            print('命令码不匹配')
        return None
    
    # 提取data，从数据的起始位置开始
    data_start_index = sof_index + 5 + 2    # 数据起始位置+帧头(5-byte)+cmd_id(2-byte)
    data_end_index = data_start_index + data_length     # data起始位置+data长度=data结束位置
    data = data[data_start_index : data_end_index]
    
    # 提取帧尾CRC16，计算期望的CRC16和真实收到的CRC16是否相似
    frame_tail_start = data_end_index   # data结束之后就是帧尾
    frame_tail_end = frame_tail_start + 2   # 帧尾长度为2-byte
    frame_tail_bytes = data[frame_tail_start : frame_tail_end]
    
    crc16 = Get_CRC16_Check_Sum(frame_header+cmd_id_bytes+data, 5+2+data_length+2)
    received_crc16 = int.from_bytes(frame_tail_bytes, byteorder='little')
    if crc16 != received_crc16:
        if info:
            print('CRC16校验失败')
        return None

    return cmd_id_bytes, data, seq

# 用来对接收双倍易伤数据的字节进行操作，提取双倍易伤机会和双倍易伤状态
# 输入的是字节数据（1个）
def radar_decision(data):
    if not isinstance(data, int) or data < 0 or data > 255:
        raise ValueError('输入的必须是一个字节(0-255)')
    # 提取第0-1位的双倍易伤机会
    double_vulnerability_chance = data & 0b00000011
    # 提取第2位的敌方双倍易伤状态
    opponent_double_vulnerability = (data & 0b00000100) >> 2
    # 提取在第3-7位的保留位
    reserved_bits = (data & 0b11111000) >> 3
    
    return double_vulnerability_chance, opponent_double_vulnerability

# 构建雷达数据部分
def build_data_radar_all(send_map, color):
    if color == 'R':
        data = bytearray()
        data.extend(bytearray(struct.pack('H', int(send_map['B1'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B1'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B2'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B2'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B3'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B3'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B4'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B4'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B5'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B5'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B7'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B7'][1]))))  # y坐标 (小端)
    else:
        data = bytearray()
        data.extend(bytearray(struct.pack('H', int(send_map['R1'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R1'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R2'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R2'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R3'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R3'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R4'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R4'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R5'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R5'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R7'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R7'][1]))))  # y坐标 (小端)
    
    return data

# 构建完整的发送数据包，和解析数据包的输入输出反过来
def build_send_packet(data, seq, cmd_id):
    data_length = len(data)     # 得到数据长度
    frame_header = bytearray([0xA5])    # SOF
    cmd_id = bytearray([cmd_id[1], cmd_id[0]])
    frame_header.extend(struct.pack('H', data_length))      # 编码数据长度（小端）
    frame_header.append(seq)
    crc8 = Get_CRC8_Check_Sum(frame_header, 4)
    frame_header.append(crc8)                   # 截止这里把帧头都封装好了
    frame_tail = bytearray()
    frame_tail.extend(struct.pack('H', Get_CRC16_Check_Sum(frame_header + cmd_id + data, len(frame_header + cmd_id + data) +1)))
    packet = frame_header + cmd_id + data + frame_tail
    return packet, (seq + 1) % 256

# 触发双倍易伤的数据构建, 输入双倍易伤机会，队伍颜色
def build_data_decision(chances, color):
    data = bytearray()
    cmd_id = [0x01, 0x21]
    cmd_id = bytearray([cmd_id[1], cmd_id[0]])
    data.extend(cmd_id)
    if color == 'R':
        data.extend(bytearray(struct.pack('H', 9)))     # 红方雷达的机器人ID
    else:
        data.extend(bytearray(struct.pack('H', 109)))   # 蓝方雷达机器人ID
    data.extend(bytearray([0x80, 0x80]))                # 双倍易伤命令码
    data.extend(bytearray(struct.pack('B', chances)))
    return data

class SerialManager:
    def __init__(self, port, baudrate=115200, color='R'):
        self.port = port
        self.baudrate = baudrate
        self.color = color
        self.ser = Serial(port, baudrate, timeout=None)
        self.seq = 0  # 添加序列号变量，初始值为0
        
    # 接收从裁判系统发送来的数据
    def receive_serial(self):
        if not self.ser:
            print("串口未打开, 无法接收线程")
            return
        
        global progress_list                    # 标记进度列表
        global double_vulnerability_chance      # 拥有双倍易伤次数
        global opponent_double_vulnerability    # 双倍易伤触发状态
        progress_cmd_id = [0x02, 0x0C]          # 任意想要接收数据的命令码，这里是雷达标记进度的命令码0x020E
        vulnerability_cmd_id = [0x02, 0x0E]     # 双倍易伤次数和触发状态
        buffer = b''                            # 初始化缓冲区
        
        while True:
            # 读取1s内收到的所有串口的数据，然后添加到缓冲区中
            received_data = self.ser.read_all()
            buffer += received_data
            # 找帧头SOF的位置(SOF = 0xA5)，如果有找到，开始解析数据包
            sof_index = buffer.find(b'\xA5')
            while sof_index != -1:
                if len(buffer) < 5:     # 至少需要5字节才能解析帧头, 数据不足，不解析，继续读取串口数据
                    break
                if len(buffer) >= sof_index +5:
                    # 从帧头开始解析数据包
                    packet_data = buffer[sof_index:]
                    # 找下一个帧头的位置，如果有找到，说明下一个帧头和当前帧头之间的数据是一个完整的数据包; 如果没有找到下一个帧头，说明不是完整的数据包，break
                    next_sof_index = packet_data.find(b'\xA5', 1)
                    if next_sof_index != -1:
                        packet_data = packet_data[:next_sof_index]
                    else:
                        break
                    
                    # 得到完整的数据包之后就可以开始解析该数据包了
                    # progress result是雷达的标记进度数据解析；vulnerability result是双倍易伤的解析
                    # 函数receive_packet返回的值有3个，即相对应的cmd_id, data, seq
                    progress_result = receive_packet(packet_data, progress_cmd_id, info=False)
                    vulnerability_result = receive_packet(packet_data, vulnerability_cmd_id, info=False)

                    # 有收到雷达标记进度数据，就把标记进度数据替换到mark_value这个list中
                    if progress_result is not None:
                        receive_cmd_id_progress, receive_data_progress, receive_seq_progress = progress_result
                        progress_list = list(receive_data_progress)      # 把收到的标记进度的数据换成列表，以便可视化
                        if self.color == 'R':
                            mark_value['B1'] = progress_list[0]
                            mark_value['B2'] = progress_list[1]
                            mark_value['B7'] = progress_list[5]
                        else:
                            mark_value['R1'] = progress_list[0]
                            mark_value['R2'] = progress_list[1]
                            mark_value['R7'] = progress_list[5]
                    # 有收到双倍易伤的数据，就调用radar_decision对双倍易伤提取，保存在已经被定义好的全局数据中
                    if vulnerability_result is not None:
                        receive_cmd_id_vulnerability, receive_data_vulnerability, receive_seq_vulnerability = vulnerability_result
                        receive_data_vulnerability = list(receive_data_vulnerability)[0]
                        double_vulnerability_chance, opponent_double_vulnerability = radar_decision(receive_data_vulnerability)
                    
                    # 从缓冲区移除已经解析好的数据包
                    buffer = buffer[sof_index + len(packet_data):]
                    # 继续寻找下一个SOF的位置（用来判断帧头）
                    sof_index = buffer.find(b'\xA5')
            
            # # 将读到的数据记录在日志文件中(未测试，不知道能不能用)
            # if hasattr(self, 'recorder') and self.recorder:
            #     self.recorder.log_received_serial_packet(packet_data, mark_value, self.seq)
            
            time.sleep(0.5)
            
    # 通过串口发送数据
    def send_serial(self, position=None):
        if not self.ser:
            print('串口未打开, 无法发送数据')
            return
        
        time_count0 = time.time()    # 计时器
        send_count = 0              # 信道占用数，上限为4
        global chances_flag  # 引用全局变量
        global double_vulnerability_chance  # 双倍易伤机会
        global opponent_double_vulnerability  # 敌方双倍易伤状态
        
        send_map = {                # 初始就是(0, 0)
            "R1": (0, 0),
            "R2": (0, 0),
            "R3": (0, 0),
            "R4": (0, 0),
            "R5": (0, 0),
            "R6": (0, 0),
            "R7": (0, 0),
            "B1": (0, 0),
            "B2": (0, 0),
            "B3": (0, 0),
            "B4": (0, 0),
            "B5": (0, 0),
            "B6": (0, 0),
            "B7": (0, 0)
        }
        
        # while True:
        #     send_count = 0      # 重置信道占用数
        #     # 更新send_map中的位置数据
        #     for robot_id, pos in position.items():
        #         if robot_id in send_map:
        #             send_map[robot_id] = pos
            
        #     try:
        #         ser_data = build_data_radar_all(send_map, self.color)
        #         packet, seq = build_send_packet(ser_data, self.seq, [0x03, 0x05])
        #         self.ser.write(packet)
        #         time.sleep(0.2)
        #         print(send_map, seq)
        #         # 如果有双倍易伤机会，并且此时不在双倍易伤状态中，就开大
        #         if double_vulnerability_chance > 0 and opponent_double_vulnerability == 0:
        #             time_count1 = time.time()
        #             # 发送时间间隔为10秒
        #             if time_count1 - time_count0 > 10:
        #                 print('准备触发双倍易伤')
        #                 data = build_data_decision(chances_flag, self.color)
        #                 packet, seq = build_send_packet(data, seq, [0x03, 0x01])
        #                 self.ser.write(packet)
        #                 print('双倍易伤触发成功', chances_flag)
        #                 # 触发成功后，更新标志位
        #                 chances_flag += 1
        #                 if chances_flag >= 3:
        #                     chances_flag = 1
                        
        #                 time_count0 = time.time()   # 重设计时器
        #     except Exception as e:
        #         print('出现错误 %s' % (e))
        # 更新send_map中的位置数据
        if position:  # 只在有位置数据时更新
            for robot_id, pos in position.items():
                # 把传入单位为m的点位，改成单位为cm的点位
                # 根据队伍颜色筛选敌方机器人
                if self.color == 'R':  # 红队只发送蓝方坐标
                    if robot_id in send_map:
                        # 四舍五入保留两位小数点
                        x = round(float(pos[0])*100, 2)
                        y = round(float(pos[1])*100, 2)
                        
                        # # 哨兵预测逻辑，蓝方B7
                        # if robot_id == 'B7':
                        #     if x > 5745.0:  # 如果蓝方哨兵在右侧区域(敌方堡垒后)
                        #         x, y = 5493.0, 7176.5  # 预测位置为蓝方哨兵启动区
                        
                        send_map[robot_id] = (x, 1500-y)
                elif self.color == 'B':  # 蓝队只发送红方坐标
                    if robot_id in send_map:
                        # 四舍五入保留两位小数点
                        x = round(float(pos[0])*100, 2)
                        y = round(float(pos[1])*100, 2)
                        
                        # # 哨兵位置预测逻辑 - 红方哨兵(R7)
                        # if robot_id == 'R7':
                        #     if x > 22255.0:  # 如果红方哨兵在右侧区域(敌方堡垒后)
                        #         x, y = 22255.0, 7823.5  # 预测位置为红方哨兵启动区
                        
                        send_map[robot_id] = (x, y)
        
        try:
            ser_data = build_data_radar_all(send_map, self.color)
            packet, self.seq = build_send_packet(ser_data, self.seq, [0x03, 0x05])      # 串口手册3.2中选手端接收小地图数据的命令码ID:0x0305
            
            # 记录数据包到日志
            if hasattr(self, 'recorder') and self.recorder:
                self.recorder.log_serial_packet(packet, position, self.seq)
            
            print(packet)
            yes = self.ser.write(packet)
            # time.sleep(0.1)
            if yes:                     # 如果真的写进ser了，就print出来
                print(send_map, self.seq)
            
            # 如果有双倍易伤机会，并且此时不在双倍易伤状态中，检查是否需要触发(被注释了,试试直接强制发)
            current_time = time.time()
            # if hasattr(self, 'last_send_time') and double_vulnerability_chance > 0 and opponent_double_vulnerability == 0:
            #     # 发送时间间隔为10秒
            #     if current_time - self.last_send_time > 10:
            #         print('准备触发双倍易伤')
            #         data = build_data_decision(chances_flag, self.color)
            #         packet, self.seq = build_send_packet(data, self.seq, [0x03, 0x01])
            #         self.ser.write(packet)
            #         print('双倍易伤触发成功', chances_flag)
            #         # 触发成功后，更新标志位
            #         chances_flag += 1
            #         if chances_flag >= 3:
            #             chances_flag = 1
                    
            #         self.last_send_time = current_time   # 重设计时器
            # else:
            #     # 首次发送，初始化时间
            #     if not hasattr(self, 'last_send_time'):
            #         self.last_send_time = current_time
                    
            # 强制一直发送触发双倍易伤的命令
            if not hasattr(self, 'last_send_time') or (current_time - self.last_send_time) >= 10:
                data = build_data_decision(chances_flag, self.color)
                packet, self.seq = build_send_packet(data, self.seq, [0x03, 0x01])
                
                # 记录双倍易伤数据包到日志
                if hasattr(self, 'recorder') and self.recorder:
                    self.recorder.log_serial_packet(packet, None, self.seq)
                
                self.ser.write(packet)
                print('双倍易伤触发成功, 标志位: ', chances_flag)
                # 触发成功后，更新标志位
                chances_flag += 1
                if chances_flag >= 3:
                    chances_flag = 1
                
                self.last_send_time = current_time   # 重设计时器
                    
        except Exception as e:
            print('出现错误 %s' % (e))
            
    def stop(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print("串口关闭成功")
            except Exception as e:
                print(f"关闭串口时出错: {e}")
