#include <thread>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <serial/serial.h>
#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <cstdint>
#include <unordered_map>

#include <algorithm> // for std::find
#include <mutex>     // for std::mutex
#include <tuple>     // for std::tuple
#include <typeinfo>  // for typeid
#include <sstream>
#include "serial.hpp"

using namespace std;
using namespace serial;

// 全局变量定义（与 header 中 extern 对应）
int double_vulnerability_chance = -1; //双倍易伤机会次数
int opponent_double_vulnerability = -1; // 是否正在触发双倍易伤
int chances_flag = 1; // 双倍易伤触发标志位，需要从1递增，每小局比赛会重置，所以每局比赛要重启程序
std::vector<int> progress_list; //标记进度列表

map<std::string,int> mark_value =
{
    {"B1",0},
    {"B2",0},
    {"B7",0},
    {"R1",0},
    {"R2",0},
    {"R7",0}
};

//机器人名字对应ID
map<std::string,int> mapping_table =
{
    {"R1", 1},
    {"R2", 2},
    {"R3", 3},
    {"R4", 4},
    {"R5", 5},
    {"R6", 6},
    {"R7", 7},
    {"B1", 101},
    {"B2", 102},
    {"B3", 103},
    {"B4", 104},
    {"B5", 105},
    {"B6", 106},
    {"B7", 107}
};

// CRC8表与 CRC16 表（定义，大小由初始化决定）
const uint8_t CRC8_INIT = 0xff;
const uint8_t CRC8_TAB[] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x5a, 0x04, 0xe6, 0xb8, 0x7b, 0x25, 0xc7, 0x99,
    0x8d, 0xd3, 0x31, 0x6f, 0xee, 0xb0, 0x52, 0x0c, 0x4d, 0x13, 0xf1, 0xad, 0x2e, 0x70, 0x92, 0xc8,
    0x10, 0x4e, 0xac, 0xf2, 0x71, 0x2f, 0xcf, 0x91, 0xd0, 0x8e, 0x6c, 0x32, 0xb1, 0xef, 0x0d, 0x53,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
    0xc4, 0x9a, 0x78, 0x26, 0xa5, 0xf9, 0x1b, 0x45, 0x04, 0x5a, 0xb8, 0xe6, 0x67, 0x39, 0xd9, 0x87,
    0x9c, 0xc2, 0x20, 0x7e, 0xff, 0xa1, 0x43, 0x1d, 0x1c, 0x42, 0xa0, 0xfe, 0x7f, 0x21, 0xc3, 0x9d,
    0x41, 0x1f, 0xfd, 0xa3, 0x22, 0x7c, 0x9e, 0xc0, 0x31, 0x6f, 0x8d, 0xd3, 0x52, 0x0c, 0xee, 0xb0,
    0x8f, 0xd1, 0x33, 0x6d, 0xe6, 0xb8, 0x5a, 0x04, 0xc7, 0x99, 0x7b, 0x25, 0xf4, 0xaa, 0x48, 0x16,
    0x09, 0x57, 0xb5, 0xef, 0x6e, 0x30, 0xd2, 0x8c, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0x85, 0xd9, 0x3b, 0x65, 0xe4, 0xba, 0x58, 0x02, 0xc1, 0x9f, 0x7d, 0x23, 0xa2, 0xfd, 0x0f, 0x51,
    0x10, 0x4e, 0xac, 0xf2, 0x71, 0x2f, 0xcf, 0x91, 0xd0, 0x8e, 0x6c, 0x32, 0xb1, 0xef, 0x0d, 0x53,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

const uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
{
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
};

// ---------------------------------------------------------
// SerialPort 成员函数实现（与 header 保持一致）
// ---------------------------------------------------------

uint8_t SerialPort::Get_CRC8_Check_Sum(const std::vector<uint8_t>& pchMessage, size_t dwLength)
{
    uint8_t ucCRC8 = CRC8_INIT;
    size_t limit = std::min(dwLength, pchMessage.size());
    for (size_t i = 0; i < limit; ++i)
    {
        ucCRC8 = CRC8_TAB[(ucCRC8 ^ pchMessage[i]) & 0xFF];
    }
    return ucCRC8;
}

uint16_t SerialPort::Get_CRC16_Check_Sum(const std::vector<uint8_t>& pchMessage, size_t dwLength)
{
    uint16_t wCRC = CRC_INIT;
    size_t limit = std::min(dwLength, pchMessage.size());
    for (size_t i = 0; i < limit; ++i)
    {
        wCRC = (wCRC >> 8) ^ wCRC_Table[(wCRC ^ pchMessage[i]) & 0xFF];
    }
    return wCRC;
}

std::tuple<std::vector<uint8_t>, std::vector<uint8_t>, uint8_t>
SerialPort::receive_packet(const std::vector<uint8_t>& data, const std::vector<uint8_t>& cmd_id, bool info)
{
    // 查找SOF
    auto sof_iter = std::find(data.begin(), data.end(), static_cast<uint8_t>(0xA5));
    if (sof_iter == data.end())
    {
        if (info) std::cout << "[receive_packet] 找不到SOF" << std::endl;
        return {{}, {}, 0};
    }
    size_t sof_index = std::distance(data.begin(), sof_iter);

    // 需要至少 5 字节帧头可用来读取 length 和 seq、crc8
    if (data.size() < sof_index + 5)
    {
        if (info) std::cout << "[receive_packet] 数据不足(帧头)" << std::endl;
        return {{}, {}, 0};
    }

    // 提取帧头
    std::vector<uint8_t> frame_header(data.begin() + sof_index, data.begin() + sof_index + 5);
    uint16_t data_length = static_cast<uint16_t>(frame_header[1]) | (static_cast<uint16_t>(frame_header[2]) << 8);

    // 检查整体是否完整：frame_header(5) + cmd_id(2) + data(data_length) + frame_tail(2)
    size_t total_length = 5 + 2 + static_cast<size_t>(data_length) + 2;
    if (data.size() < sof_index + total_length)
    {
        if (info) std::cout << "[receive_packet] 数据不足(整个包) need " << total_length << " bytes, have " << (data.size()-sof_index) << std::endl;
        return {{}, {}, 0};
    }

    // CRC8 校验（前 4 字节）
    std::vector<uint8_t> header_crc_input(frame_header.begin(), frame_header.begin() + 4);
    if (Get_CRC8_Check_Sum(header_crc_input, 4) != frame_header[4])
    {
        if (info) std::cout << "[receive_packet] CRC8校验失败" << std::endl;
        return {{}, {}, 0};
    }

    // 提取 cmd_id（包中为小端存储）
    std::vector<uint8_t> cmd_id_bytes(data.begin() + sof_index + 5, data.begin() + sof_index + 7);
    std::vector<uint8_t> expected_cmd_id_le = {cmd_id[1], cmd_id[0]};
    if (cmd_id_bytes != expected_cmd_id_le)
    {
        if (info) std::cout << "[receive_packet] 命令码不匹配" << std::endl;
        return {{}, {}, 0};
    }

    // 提取 data_field
    size_t data_start_index = sof_index + 5 + 2;
    size_t data_end_index = data_start_index + data_length;
    std::vector<uint8_t> data_field(data.begin() + data_start_index, data.begin() + data_end_index);

    // 提取帧尾 CRC16（小端）
    size_t frame_tail_start = data_end_index;
    uint16_t received_crc16 = static_cast<uint16_t>(data[frame_tail_start]) | (static_cast<uint16_t>(data[frame_tail_start + 1]) << 8);

    // 计算 CRC16：frame_header + cmd_id + data_field
    std::vector<uint8_t> crc16_input;
    crc16_input.insert(crc16_input.end(), frame_header.begin(), frame_header.end());
    crc16_input.insert(crc16_input.end(), cmd_id_bytes.begin(), cmd_id_bytes.end());
    crc16_input.insert(crc16_input.end(), data_field.begin(), data_field.end());

    uint16_t calculated_crc16 = Get_CRC16_Check_Sum(crc16_input, crc16_input.size());
    if (calculated_crc16 != received_crc16)
    {
        if (info) std::cout << "[receive_packet] CRC16校验失败" << std::endl;
        return {{}, {}, 0};
    }

    uint8_t seq = frame_header[3];
    return {cmd_id_bytes, data_field, seq};
}

RadarDecision SerialPort::radar_decision(uint8_t data)
{
    uint8_t double_vul_chance = data & 0b00000011;
    uint8_t opponent_double_vul = (data & 0b00000100) >> 2;
    uint8_t reserved_bits = (data & 0b11111000) >> 3;
    return {double_vul_chance, opponent_double_vul, reserved_bits};
}

void SerialPort::append_uint16_t_le(std::vector<uint8_t>& data, uint16_t value)
{
    data.push_back(static_cast<uint8_t>(value & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void SerialPort::append_uint16_le(std::vector<uint8_t>& data, uint16_t value)
{
    append_uint16_t_le(data, value);
}

std::vector<uint8_t> SerialPort::build_data_radar_all(const std::unordered_map<std::string, std::pair<float,float>>& send_map, char color)
{
    std::vector<uint8_t> data;
    if (color == 'R')
    {
        const char* keys[] = {"B1", "B2", "B3", "B4", "B5", "B7"};
        for (const auto& key : keys)
        {
            auto it = send_map.find(key);
            if (it == send_map.end()) { append_uint16_t_le(data, 0); append_uint16_t_le(data, 0); continue; }
            append_uint16_t_le(data, static_cast<uint16_t>(it->second.first));
            append_uint16_t_le(data, static_cast<uint16_t>(it->second.second));
        }
    }
    else
    {
        const char* keys[] = {"R1", "R2", "R3", "R4", "R5", "R7"};
        for (const auto& key : keys)
        {
            auto it = send_map.find(key);
            if (it == send_map.end()) { append_uint16_t_le(data, 0); append_uint16_t_le(data, 0); continue; }
            append_uint16_t_le(data, static_cast<uint16_t>(it->second.first));
            append_uint16_t_le(data, static_cast<uint16_t>(it->second.second));
        }
    }
    return data;
}

std::pair<std::vector<uint8_t>, uint8_t> SerialPort::build_send_packet(const std::vector<uint8_t>& data, const std::vector<uint8_t>& cmd_id, uint8_t& seq)
{
    uint16_t data_length = static_cast<uint16_t>(data.size());
    std::vector<uint8_t> frame_header;
    frame_header.push_back(0xA5);

    append_uint16_le(frame_header, data_length);

    frame_header.push_back(seq);

    uint8_t crc8 = Get_CRC8_Check_Sum(frame_header, 4);
    frame_header.push_back(crc8);

    std::vector<uint8_t> cmd_id_le = {cmd_id[1], cmd_id[0]};

    std::vector<uint8_t> packet(frame_header);
    packet.insert(packet.end(), cmd_id_le.begin(), cmd_id_le.end());
    packet.insert(packet.end(), data.begin(), data.end());

    uint16_t crc16 = Get_CRC16_Check_Sum(packet, packet.size());
    append_uint16_le(packet, crc16);

    uint8_t next_seq = static_cast<uint8_t>((seq + 1) % 256);
    return {packet, next_seq};
}

std::vector<uint8_t> SerialPort::build_data_decision(uint8_t chances, char &color)
{
    std::vector<uint8_t> data;
    data.push_back(0x21);
    data.push_back(0x01);

    if (color == 'R')
    {
        append_uint16_le(data, 9);
    }
    else
    {
        append_uint16_le(data, 109);
    }

    data.push_back(0x80);
    data.push_back(0x80);

    data.push_back(chances);
    return data;
}

// ---------------------------------------------------------
// SerialManager 实现（改进过 receive_serial，增强边界检查与日志）
// ---------------------------------------------------------

SerialManager::SerialManager(const std::string& port, int bandrate, char color)
    : port_(port), bandrate_(bandrate), color_(color), seq_(0)
{
    last_send_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(11);
}

SerialManager::~SerialManager()
{
    std::cout << "serialmanager is close" << std::endl;
}

bool SerialManager::serial_set()
{
    try
    {
        ser.setPort(port_);
        ser.setBaudrate(bandrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        ser.open();
        if (!ser.isOpen())
        {
            std::cout << "串口设置失败" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "串口打开异常: " << e.what() << std::endl;
        return false;
    }
    return true;
}

void SerialManager::receive_serial()
{
    std::vector<uint8_t> buffer;
    std::vector<uint8_t> progress_cmd_id = {0x0C, 0x02};
    std::vector<uint8_t> vulnerability_cmd_id = {0x0E, 0x01};

    std::cout << "[receive_serial] 开始接收数据..." << std::endl;

    while (true)
    {
        try
        {
            // 读取所有可用字节
            std::vector<uint8_t> receiver_data = serial_read_all();
            if (!receiver_data.empty())
            {
                buffer.insert(buffer.end(), receiver_data.begin(), receiver_data.end());
            }

            // 循环尝试从 buffer 中解析尽可能多的完整包
            while (true)
            {
                // 查找 SOF
                auto sof_iter = std::find(buffer.begin(), buffer.end(), static_cast<uint8_t>(0xA5));
                if (sof_iter == buffer.end()) break;
                size_t sof_index = std::distance(buffer.begin(), sof_iter);

                // 需要至少 5 字节帧头
                if (buffer.size() < sof_index + 5) break;

                // 读取 data_length（小端）
                uint16_t data_length = static_cast<uint16_t>(buffer[sof_index + 1]) | (static_cast<uint16_t>(buffer[sof_index + 2]) << 8);
                size_t total_packet_len = 5 + 2 + static_cast<size_t>(data_length) + 2;

                // 如果整个包未到齐，等待更多数据
                if (buffer.size() < sof_index + total_packet_len) break;

                // 现在 packet_data 是一个完整的包（从 SOF 开始，长度 total_packet_len）
                std::vector<uint8_t> packet_data(buffer.begin() + sof_index, buffer.begin() + sof_index + total_packet_len);

                // 解析两个我们关心的命令码（分别解析进度包和易伤包）
                auto progress_result = receive_packet(packet_data, progress_cmd_id, false);
                auto vulnerability_result = receive_packet(packet_data, vulnerability_cmd_id, false);

                // 如果 progress_result 有数据（第二项非空）
                if (std::get<1>(progress_result).size() > 0)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    progress_list.clear();
                    for (auto v : std::get<1>(progress_result)) progress_list.push_back(v);

                    // 防御性地访问索引
                    if (color_ == 'R')
                    {
                        if (progress_list.size() > 0) mark_value["B1"] = progress_list[0];
                        if (progress_list.size() > 1) mark_value["B2"] = progress_list[1];
                        if (progress_list.size() > 5) mark_value["B7"] = progress_list[5];
                    }
                    else
                    {
                        if (progress_list.size() > 0) mark_value["R1"] = progress_list[0];
                        if (progress_list.size() > 1) mark_value["R2"] = progress_list[1];
                        if (progress_list.size() > 5) mark_value["R7"] = progress_list[5];
                    }
                }

                // 易伤包解析
                if (std::get<1>(vulnerability_result).size() > 0)
                {
                    uint8_t b = std::get<1>(vulnerability_result)[0];
                    auto res = radar_decision(b);
                    double_vulnerability_chance = res.double_vulnerability_chance;
                    opponent_double_vulnerability = res.opponent_double_vulnerability;
                }

                // 从 buffer 中移除已处理的完整包（从包头 SOF 开始）
                buffer.erase(buffer.begin(), buffer.begin() + sof_index + total_packet_len);
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[receive_serial] 异常: " << e.what() << std::endl;
            // 不退出循环；继续重试读取
        }
        // 降低 CPU 占用并给其他线程机会
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void SerialManager::send_serial(const std::unordered_map<std::string, std::pair<float,float>>* position)
{
    std::unordered_map<std::string, std::pair<float,float>> send_map =
    {
        {"R1",{0.0, 0.0}}, {"R2",{0.0, 0.0}}, {"R3",{0.0, 0.0}}, {"R4",{0.0, 0.0}}, {"R5",{0.0, 0.0}}, {"R6",{0.0, 0.0}}, {"R7",{0.0, 0.0}},
        {"B1",{0.0, 0.0}}, {"B2",{0.0, 0.0}}, {"B3",{0.0, 0.0}}, {"B4",{0.0, 0.0}}, {"B5",{0.0, 0.0}}, {"B6",{0.0, 0.0}}, {"B7",{0.0, 0.0}}
    };
    if (position)
    {
        for (const auto& kv : *position)
        {
            const std::string& robot_id = kv.first;
            const auto& pos = kv.second;
            int x = static_cast<int>(pos.first * 100 + 0.5f);
            int y = static_cast<int>(pos.second * 100 + 0.5f);
            if (send_map.count(robot_id))
            {
                if (color_ == 'R')
                {
                    send_map[robot_id] = {x, 1500 - y};
                }
                else
                {
                    send_map[robot_id] = {x, y};
                }
            }
        }
    }

    try
    {
        auto ser_data = build_data_radar_all(send_map, color_);
        auto [packet, next_seq] = build_send_packet(ser_data, {0x03, 0x05}, seq_);
        seq_ = next_seq;
        if (ser.isOpen())
        {
            //size_t bytes_written = ser.write(packet);
            std::cout << "[send_serial] 数据包写入: " << packet.size() << " bytes" << std::endl;
        }
        else
        {
            std::cerr << "[send_serial] 串口未打开，无法发送" << std::endl;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_send_time_).count() >= 10)
        {
            auto data = build_data_decision(static_cast<uint8_t>(chances_flag), color_);
            auto [chance_packet, chance_next_seq] = build_send_packet(data, {0x03, 0x05}, seq_);
            seq_ = chance_next_seq;
            if (ser.isOpen())
            {
                size_t chance_written = ser.write(chance_packet);
                std::cout << "[send_serial] 触发包写入: " << chance_written << " bytes" << std::endl;
            }
            else
            {
                std::cerr << "[send_serial] 串口未打开，无法发送触发包" << std::endl;
            }
            std::cout << "[send_serial] 双倍易伤触发成功, 标志位: " << chances_flag << std::endl;
            chances_flag++;
            if (chances_flag >= 3) chances_flag = 1;
            last_send_time_ = now;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[send_serial] 出现错误: " << e.what() << std::endl;
    }
}

void SerialManager::stop()
{
    try
    {
        if (ser.isOpen()) ser.close();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[stop] 关闭串口异常: " << e.what() << std::endl;
    }
    std::cout << "串口关闭成功" << std::endl;
}

std::vector<uint8_t> SerialManager::serial_read_all()
{
    std::vector<uint8_t> out;
    try
    {
        if (!ser.isOpen()) return out;
        size_t avail = ser.available();
        if (avail == 0) return out;
        std::string s = ser.read(avail);
        out.assign(s.begin(), s.end());
    }
    catch (const std::exception& e)
    {
        std::cerr << "[serial_read_all] 异常: " << e.what() << std::endl;
    }
    return out;
}

// EOF