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
#include <cmath>      // for std::lround
#include <unordered_map>

#include <algorithm> // for std::find
#include <mutex>     // for std::mutex
#include <tuple>     // for std::tuple
#include <sstream>   // for std::istringstream
#include <iomanip>   // for std::setw, std::setfill
#include <fstream>   // for std::ofstream (JSON log)

#include "serial.hpp"


// 全局变量定义（与 header 中 extern 对应）
int double_vulnerability_chance = -1; //双倍易伤机会次数
int opponent_double_vulnerability = -1; // 是否正在触发双倍易伤
int chances_flag = 1; // 双倍易伤触发标志位，需要从1递增，每小局比赛会重置，所以每局比赛要重启程序
std::vector<int> progress_list; //标记进度列表
int key_staus = 0;
int wave_key_received = 0;
int encryption_level = 1;
int key_modifiable = 0;
int game_type = 0;
int game_progress = 0;
int stage_remain_time = 0;
uint64_t sync_timestamp = 0;

// JSON 日志记录辅助函数
static std::string bytes_to_hex(const std::vector<uint8_t>& data)
{
    std::ostringstream oss;
    for (size_t i = 0; i < data.size(); ++i)
    {
        if (i > 0) oss << " ";
        oss << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(data[i]);
    }
    oss << std::dec;
    return oss.str();
}

// 从数据包字节中提取 cmd_id（小端，字节 5-6）
static std::string extract_cmd_id_hex(const std::vector<uint8_t>& packet)
{
    if (packet.size() < 7) return "0000";
    uint16_t cmd_id = static_cast<uint16_t>(packet[5]) | (static_cast<uint16_t>(packet[6]) << 8);
    std::ostringstream oss;
    oss << std::hex << std::setw(4) << std::setfill('0') << cmd_id;
    return oss.str();
}

// 从数据包字节中提取 seq（字节 3）
static int extract_seq(const std::vector<uint8_t>& packet)
{
    if (packet.size() < 5) return -1;
    return static_cast<int>(packet[3]);
}

static void log_packet_json(const std::string& filename,
                            const std::vector<uint8_t>& packet,
                            const std::string& direction,
                            const std::string& status = "ok")
{
    std::ofstream ofs(filename, std::ios::app);
    if (!ofs.is_open())
    {
        std::cerr << "[log] 无法打开日志文件: " << filename << std::endl;
        return;
    }

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    ofs << "{\"ts\":" << ms
        << ",\"dir\":\"" << direction << "\""
        << ",\"cmd_id\":\"0x" << extract_cmd_id_hex(packet) << "\""
        << ",\"seq\":" << extract_seq(packet)
        << ",\"size\":" << packet.size()
        << ",\"status\":\"" << status << "\""
        << ",\"hex\":\"" << bytes_to_hex(packet) << "\"}\n";
}

// 记录非数据包事件（丢弃字节、错误等）
static void log_event(const std::string& filename,
                      const std::string& event,
                      const std::string& detail = "")
{
    std::ofstream ofs(filename, std::ios::app);
    if (!ofs.is_open()) return;

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    ofs << "{\"ts\":" << ms
        << ",\"event\":\"" << event << "\"";
    if (!detail.empty())
        ofs << ",\"detail\":\"" << detail << "\"";
    ofs << "}\n";
}

struct radar_ai
{
    uint8_t radar_cmd;  // double_vulnerability_chance
    uint8_t password_cmd; // 1 or 2
    uint8_t password_1; // ascii
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
};

// struct pos_info
// {
//     int8_t 
// };




std::map<std::string,int> mark_value =
{
    {"B1",0},
    {"B2",0},
    {"B7",0},
    {"R1",0},
    {"R2",0},
    {"R7",0}
};

//机器人名字对应ID
std::map<std::string,int> mapping_table =
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
const uint8_t CRC8_TAB[256] =
{
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
};

// 0x0305 数据包结构体 (48 bytes = 24 fields × 2 bytes each)
// 对方 (bytes 0-23) + 己方 (bytes 24-47)
struct PosData
{
    // 对方 (enemy) — bytes 0-23
    int16_t enemy_hero_x;
    int16_t enemy_hero_y;
    int16_t enemy_engineer_x;
    int16_t enemy_engineer_y;
    int16_t enemy_standard_3_x;
    int16_t enemy_standard_3_y;
    int16_t enemy_standard_4_x;
    int16_t enemy_standard_4_y;
    int16_t enemy_air_force_x;
    int16_t enemy_air_force_y;
    int16_t enemy_sentry_x;
    int16_t enemy_sentry_y;
    // 己方 (our) — bytes 24-47
    int16_t our_hero_x;
    int16_t our_hero_y;
    int16_t our_engineer_x;
    int16_t our_engineer_y;
    int16_t our_standard_3_x;
    int16_t our_standard_3_y;
    int16_t our_standard_4_x;
    int16_t our_standard_4_y;
    int16_t our_air_force_x;
    int16_t our_air_force_y;
    int16_t our_sentry_x;
    int16_t our_sentry_y;
};

void print_pos_data(const std::vector<uint8_t>& data)
{
    if (data.size() < 48) { std::cout << "[DATA] 数据不足 48 字节, 实际=" << data.size() << std::endl; return; }
    const uint8_t* d = data.data();
    auto read_i16 = [&](size_t off) -> int16_t {
        return static_cast<int16_t>(d[off] | (d[off + 1] << 8));
    };
    std::cout << "[DATA] === 对方 (enemy) ===" << std::endl;
    std::cout << "[DATA]   hero:       (" << read_i16(0)  << "," << read_i16(2)  << ")" << std::endl;
    std::cout << "[DATA]   engineer:   (" << read_i16(4)  << "," << read_i16(6)  << ")" << std::endl;
    std::cout << "[DATA]   standard_3: (" << read_i16(8)  << "," << read_i16(10) << ")" << std::endl;
    std::cout << "[DATA]   standard_4: (" << read_i16(12) << "," << read_i16(14) << ")" << std::endl;
    std::cout << "[DATA]   air_force:  (" << read_i16(16) << "," << read_i16(18) << ")" << std::endl;
    std::cout << "[DATA]   sentry:     (" << read_i16(20) << "," << read_i16(22) << ")" << std::endl;
    std::cout << "[DATA] === 己方 (our) ===" << std::endl;
    std::cout << "[DATA]   hero:       (" << read_i16(24) << "," << read_i16(26) << ")" << std::endl;
    std::cout << "[DATA]   engineer:   (" << read_i16(28) << "," << read_i16(30) << ")" << std::endl;
    std::cout << "[DATA]   standard_3: (" << read_i16(32) << "," << read_i16(34) << ")" << std::endl;
    std::cout << "[DATA]   standard_4: (" << read_i16(36) << "," << read_i16(38) << ")" << std::endl;
    std::cout << "[DATA]   air_force:  (" << read_i16(40) << "," << read_i16(42) << ")" << std::endl;
    std::cout << "[DATA]   sentry:     (" << read_i16(44) << "," << read_i16(46) << ")" << std::endl;
}

void check_packet(const std::vector<uint8_t>& packet)
{
    std::cout << "[PACKET] hex(" << packet.size() << "B): ";
    for (size_t i = 0; i < packet.size(); i++)
    {
        std::cout << std::hex << static_cast<int>(packet[i]) << " ";
        if ((i + 1) % 16 == 0 && i + 1 < packet.size())
            std::cout << std::endl << "        ";
    }
    std::cout << std::dec << std::endl;
    size_t data_ofs = 7; // frame_header(5) + cmd_id(2)
    size_t data_len = packet.size() > data_ofs + 2 ? packet.size() - data_ofs - 2 : 0;
    if (data_len >= 48)
    {
        std::vector<uint8_t> data_part(packet.begin() + data_ofs, packet.begin() + data_ofs + 48);
        std::cout << "[PACKET] data_size=" << 48 << " (24 fields × 2 bytes)" << std::endl;
        print_pos_data(data_part);
    }
    else if (data_len > 0)
    {
        std::cout << "[PACKET] data_size=" << data_len << " bytes (non-standard)" << std::endl;
    }
}

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
    crc16_input.reserve(frame_header.size() + cmd_id_bytes.size() + data_field.size());
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

std::tuple<std::vector<uint8_t>, std::vector<uint8_t>, uint8_t>
SerialPort::receive_any_packet(const std::vector<uint8_t>& data, bool info)
{
    // 查找SOF
    auto sof_iter = std::find(data.begin(), data.end(), static_cast<uint8_t>(0xA5));
    if (sof_iter == data.end())
    {
        if (info) std::cout << "[receive_any] 找不到SOF" << std::endl;
        return {{}, {}, 0};
    }
    size_t sof_index = std::distance(data.begin(), sof_iter);

    // 需要至少 5 字节帧头
    if (data.size() < sof_index + 5)
    {
        if (info) std::cout << "[receive_any] 数据不足(帧头)" << std::endl;
        return {{}, {}, 0};
    }

    // 提取帧头
    std::vector<uint8_t> frame_header(data.begin() + sof_index, data.begin() + sof_index + 5);
    uint16_t data_length = static_cast<uint16_t>(frame_header[1]) | (static_cast<uint16_t>(frame_header[2]) << 8);

    // 检查整体是否完整
    size_t total_length = 5 + 2 + static_cast<size_t>(data_length) + 2;
    if (data.size() < sof_index + total_length)
    {
        if (info) std::cout << "[receive_any] 数据不足(整个包) need " << total_length
                            << " bytes, have " << (data.size() - sof_index) << std::endl;
        return {{}, {}, 0};
    }

    // CRC8 校验
    std::vector<uint8_t> header_crc_input(frame_header.begin(), frame_header.begin() + 4);
    if (Get_CRC8_Check_Sum(header_crc_input, 4) != frame_header[4])
    {
        if (info) std::cout << "[receive_any] CRC8校验失败" << std::endl;
        return {{}, {}, 0};
    }

    // 提取 cmd_id（不筛选，原样返回小端字节序）
    std::vector<uint8_t> cmd_id_bytes(data.begin() + sof_index + 5, data.begin() + sof_index + 7);

    // 提取 data_field
    size_t data_start_index = sof_index + 5 + 2;
    size_t data_end_index = data_start_index + data_length;
    std::vector<uint8_t> data_field(data.begin() + data_start_index, data.begin() + data_end_index);

    // CRC16 校验
    size_t frame_tail_start = data_end_index;
    uint16_t received_crc16 = static_cast<uint16_t>(data[frame_tail_start]) | (static_cast<uint16_t>(data[frame_tail_start + 1]) << 8);

    std::vector<uint8_t> crc16_input;
    crc16_input.reserve(frame_header.size() + cmd_id_bytes.size() + data_field.size());
    crc16_input.insert(crc16_input.end(), frame_header.begin(), frame_header.end());
    crc16_input.insert(crc16_input.end(), cmd_id_bytes.begin(), cmd_id_bytes.end());
    crc16_input.insert(crc16_input.end(), data_field.begin(), data_field.end());

    uint16_t calculated_crc16 = Get_CRC16_Check_Sum(crc16_input, crc16_input.size());
    if (calculated_crc16 != received_crc16)
    {
        if (info) std::cout << "[receive_any] CRC16校验失败" << std::endl;
        return {{}, {}, 0};
    }

    uint8_t seq = frame_header[3];
    return {cmd_id_bytes, data_field, seq};
}

void SerialPort::print_received_data(const std::vector<uint8_t>& cmd_id_bytes,
                                      const std::vector<uint8_t>& data_field,
                                      uint8_t seq)
{
    uint16_t cmd_id = static_cast<uint16_t>(cmd_id_bytes[1]) << 8 | static_cast<uint16_t>(cmd_id_bytes[0]);

    std::cout << "\n===== [接收数据包] =====" << std::endl;
    std::cout << "  命令码: 0x" << std::hex << std::setfill('0') << std::setw(4) << cmd_id << std::dec << std::endl;
    std::cout << "  序列号: " << static_cast<int>(seq) << std::endl;
    std::cout << "  数据长度: " << data_field.size() << " bytes" << std::endl;
    std::cout << "  数据内容(hex): ";
    for (auto byte : data_field)
        std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte) << " ";
    std::cout << std::dec << std::endl;
    std::cout << "========================\n" << std::endl;
}

RadarDecision SerialPort::radar_decision(uint8_t data)
{
    uint8_t double_vul_chance = data & 0b00000011;        // bit 0-1
    uint8_t opponent_double_vul = (data & 0b00000100) >> 2; // bit 2
    uint8_t encryption_lvl = (data & 0b00011000) >> 3;     // bit 3-4
    uint8_t key_mod = (data & 0b00100000) >> 5;            // bit 5
    uint8_t reserved = (data & 0b11000000) >> 6;           // bit 6-7
    return {double_vul_chance, opponent_double_vul, encryption_lvl, key_mod, reserved};
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
    // 0x0305 协议: 对方(英雄,工程,3号步兵,4号步兵,6号空中,哨兵) + 己方(同) = 48 字节
    static const std::string enemy_keys_r[] = {"B1", "B2", "B3", "B4", "B6", "B7"}; // 红方视角: 对方=蓝
    static const std::string our_keys_r[]   = {"R1", "R2", "R3", "R4", "R6", "R7"}; // 红方视角: 己方=红
    static const std::string enemy_keys_b[] = {"R1", "R2", "R3", "R4", "R6", "R7"}; // 蓝方视角: 对方=红
    static const std::string our_keys_b[]   = {"B1", "B2", "B3", "B4", "B6", "B7"}; // 蓝方视角: 己方=蓝

    const std::string* enemy_keys = (color == 'R') ? enemy_keys_r : enemy_keys_b;
    const std::string* our_keys   = (color == 'R') ? our_keys_r   : our_keys_b;

    std::vector<uint8_t> data;
    data.reserve(48);

    auto write_coord = [&](const std::string& key, bool /*unused*/) {
        auto it = send_map.find(key);
        if (it == send_map.end()) { append_uint16_t_le(data, 0); append_uint16_t_le(data, 0); return; }
        append_uint16_t_le(data, static_cast<uint16_t>(std::lround(it->second.first)));
        append_uint16_t_le(data, static_cast<uint16_t>(std::lround(it->second.second)));
    };

    // 对方 6 台 (bytes 0-23)
    for (int i = 0; i < 6; i++) write_coord(enemy_keys[i], false);
    // 己方 6 台 (bytes 24-47)
    for (int i = 0; i < 6; i++) write_coord(our_keys[i], false);

    return data;
}

std::pair<std::vector<uint8_t>, uint8_t> SerialPort::build_send_packet(const std::vector<uint8_t>& data, const std::vector<uint8_t>& cmd_id, uint8_t& seq)
{
    uint16_t data_length = static_cast<uint16_t>(data.size());
    std::vector<uint8_t> frame_header;
    frame_header.reserve(5);
    frame_header.push_back(0xA5);

    append_uint16_le(frame_header, data_length);

    frame_header.push_back(seq);

    uint8_t crc8 = Get_CRC8_Check_Sum(frame_header, 4);
    frame_header.push_back(crc8);

    std::vector<uint8_t> cmd_id_le = {cmd_id[1], cmd_id[0]};

    std::vector<uint8_t> packet(frame_header);
    packet.reserve(frame_header.size() + 2 + data.size() + 2);
    packet.insert(packet.end(), cmd_id_le.begin(), cmd_id_le.end());
    packet.insert(packet.end(), data.begin(), data.end());

    uint16_t crc16 = Get_CRC16_Check_Sum(packet, packet.size());
    append_uint16_le(packet, crc16);

    uint8_t next_seq = static_cast<uint8_t>((seq + 1) % 256);
    return {packet, next_seq};
}

std::vector<uint8_t> SerialPort::build_data_decision(uint8_t chances, char color)
{
    std::vector<uint8_t> data;
    data.reserve(7);
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

SerialManager::SerialManager(const std::string& port, int bandrate, char color, bool debug)
    : port_(port), bandrate_(bandrate), color_(color), seq_(0), debug_mode_(debug)
{
    last_send_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(11);
    trigger_window_start_ = std::chrono::steady_clock::now() - std::chrono::seconds(31);
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
    std::vector<uint8_t> progress_cmd_id = {0x02, 0x0C};
    std::vector<uint8_t> vulnerability_cmd_id = {0x02, 0x0E};
    std::vector<uint8_t> game_status_cmd_id = {0x00, 0x01};

    std::cout << "[receive_serial] 开始接收数据..." << std::endl;

    while (running_)
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

                // 记录 SOF 之前的丢弃字节
                if (sof_index > 0)
                {
                    std::vector<uint8_t> dropped(buffer.begin(), buffer.begin() + sof_index);
                    log_event("serial_rx_log.json", "dropped", bytes_to_hex(dropped));
                }

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
                    // 实际数据为 6 字节独立值，每个机器人 1 字节进度标记 (与 Python 原版一致)
                    progress_list.clear();
                    for (auto v : std::get<1>(progress_result)) progress_list.push_back(v);

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

                // 调试模式: 打印 0x020C 接收数据
                if (debug_mode_ && std::get<1>(progress_result).size() > 0)
                {
                    print_received_data(std::get<0>(progress_result),
                                        std::get<1>(progress_result),
                                        std::get<2>(progress_result));
                }

                // 易伤包解析 (0x020E 雷达自主决策信息同步: 双倍易伤 + 干扰等级)
                if (std::get<1>(vulnerability_result).size() > 0)
                {
                    uint8_t b = std::get<1>(vulnerability_result)[0];
                    auto res = radar_decision(b);
                    std::lock_guard<std::mutex> lock(mutex_);
                    double_vulnerability_chance = res.double_vulnerability_chance;
                    opponent_double_vulnerability = res.opponent_double_vulnerability;
                    encryption_level = res.encryption_level;
                    key_modifiable = res.key_modifiable;

                    // 调试模式: 打印 0x020E 接收数据
                    if (debug_mode_)
                    {
                        print_received_data(std::get<0>(vulnerability_result),
                                            std::get<1>(vulnerability_result),
                                            std::get<2>(vulnerability_result));
                    }
                }

                // 雷达无线链路 — 密钥数据解析 (0x0A06)
                {
                    static std::vector<uint8_t> key_cmd  = {0x0A, 0x06};

                    auto key_result = receive_packet(packet_data, key_cmd, false);
                    if (std::get<1>(key_result).size() >= 6)
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        wave_key_received = 1;
                        key_staus = 1;
                    }
                }

                // 比赛状态数据解析 (0x0001: game_status)
                // data layout: byte0(game_type:4|game_progress:4) + uint16(stage_remain_time) + uint64(SyncTimeStamp)
                {
                    auto game_result = receive_packet(packet_data, game_status_cmd_id, false);
                    const auto& data_field = std::get<1>(game_result);
                    if (data_field.size() >= 1)
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        uint8_t status_byte = data_field[0];
                        game_type = status_byte & 0x0F;
                        game_progress = (status_byte >> 4) & 0x0F;
                        if (data_field.size() >= 3)
                            stage_remain_time = static_cast<uint16_t>(data_field[1]) | (static_cast<uint16_t>(data_field[2]) << 8);
                        if (data_field.size() >= 11)
                            sync_timestamp = static_cast<uint64_t>(data_field[3]) | (static_cast<uint64_t>(data_field[4]) << 8) | (static_cast<uint64_t>(data_field[5]) << 16) | (static_cast<uint64_t>(data_field[6]) << 24) | (static_cast<uint64_t>(data_field[7]) << 32) | (static_cast<uint64_t>(data_field[8]) << 40) | (static_cast<uint64_t>(data_field[9]) << 48) | (static_cast<uint64_t>(data_field[10]) << 56);
                    }

                    if (debug_mode_ && data_field.size() >= 1)
                    {
                        print_received_data(std::get<0>(game_result),
                                            data_field,
                                            std::get<2>(game_result));
                        std::cout << "[0x0001] game_type=" << game_type
                                  << " game_progress=" << game_progress
                                  << " remain_time=" << stage_remain_time << "s"
                                  << " sync_ts=" << sync_timestamp << std::endl;
                    }
                }

                // 记录接收包到日志（含 CRC 校验状态）
                {
                    auto any_result = receive_any_packet(packet_data, false);
                    std::string status = std::get<1>(any_result).empty() ? "crc_fail" : "ok";
                    log_packet_json("serial_rx_log.json", packet_data, "rx", status);
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
    // Merge incoming positions into persistent cache (retain last known if not updated)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (position)
        {
            for (const auto& kv : *position)
            {
                const std::string& robot_id = kv.first;
                const auto& pos = kv.second;
                float x = pos.first;
                float y = pos.second;
                pos_cache_[robot_id] = {x, y};
            }
        }
    }

    try
    {
        // 比赛阶段=4(比赛中) 时持续发送坐标数据包，支持所有对抗赛类型(1-5)
        if (game_type >= 1 && game_type <= 5 && game_progress == 4)
        {
            std::vector<uint8_t> packet;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                auto ser_data = build_data_radar_all(pos_cache_, color_);
                auto result = build_send_packet(ser_data, {0x03, 0x05}, seq_);
                packet = std::move(result.first);
                seq_ = result.second;
            }

            if (debug_mode_)
            {
                std::cout << "\n========== [DEBUG] 发送位置数据包 (0x0305) ==========" << std::endl;
                std::cout << "[DEBUG] 数据域大小: " << (packet.size() - 9) << " bytes" << std::endl;
                std::cout << "[DEBUG] 总包大小: " << packet.size() << " bytes" << std::endl;
                std::cout << "[DEBUG] 完整数据包(hex): ";
                for (auto byte : packet) std::cout << std::hex << static_cast<int>(byte) << " ";
                std::cout << std::dec << std::endl;
                std::cout << "========================================\n" << std::endl;
            }
            else
            {
                check_packet(packet);
            }

            log_packet_json("serial_tx_log.json", packet, "tx");
            if (ser.isOpen())
            {
                size_t bytes_written = ser.write(packet);
                std::cout << "[send_serial] 数据包写入: " << bytes_written << " bytes" << std::endl;
            }
            else
            {
                std::cerr << "[send_serial] 串口未打开，无法发送" << std::endl;
            }
        }

        last_send_time_ = std::chrono::steady_clock::now();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[send_serial] 出现错误: " << e.what() << std::endl;
    }
}

void SerialManager::send_serial_key(uint8_t /*id1*/, uint8_t id2, const std::string& key_str)
{
    // 己方密钥表: 三个六位密钥
    static const std::string own_keys[3] = {
        "123456",
        "654321",
        "111222"
    };
    static uint8_t key_id_counter = 0;

    uint8_t current_id = key_id_counter;
    key_id_counter = (key_id_counter + 1) % 3;

    try
    {
        // id2: 指令类型 (1=更新己方密钥, 2=验证破解密钥)
        const std::string& selected_key = (id2 == 1) ? own_keys[current_id] : key_str;

        std::vector<uint8_t> data;
        data.push_back(current_id);  // byte 0: 标识符 (双倍易伤数据, 0/1/2 轮切)
        data.push_back(id2);  // byte 1: 指令
        for (size_t i = 0; i < 6; i++)
        {
            if (i < selected_key.size())
                data.push_back(static_cast<uint8_t>(selected_key[i]));
            else
                data.push_back(0);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        auto [packet, next_seq] = build_send_packet(data, {0x0A, 0x06}, seq_);
        seq_ = next_seq;

        log_packet_json("serial_tx_log.json", packet, "tx");
        if (ser.isOpen())
        {
            size_t bytes_written = ser.write(packet);
            std::cout << "[send_serial_key] 密钥包写入: " << bytes_written << " bytes, 指令="
                      << static_cast<int>(id2) << std::endl;
        }
        else
        {
            std::cerr << "[send_serial_key] 串口未打开，无法发送密钥" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[send_serial_key] 出现错误: " << e.what() << std::endl;
    }
}

void SerialManager::stop()
{
    running_ = false;
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

void SerialManager::set_debug_mode(bool mode)
{
    debug_mode_ = mode;
    std::cout << "[set_debug_mode] 调试模式: " << (debug_mode_ ? "开启" : "关闭") << std::endl;
}

void SerialManager::manual_debug_send()
{
    if (!debug_mode_)
    {
        std::cout << "[manual_debug_send] 请先开启调试模式 (set_debug_mode(true))" << std::endl;
        return;
    }

    std::cout << "\n========== 手动调试发送模式 ==========" << std::endl;
    std::cout << "请选择发送类型:" << std::endl;
    std::cout << "  1. 发送位置数据包 (0x0305)" << std::endl;
    std::cout << "  2. 发送触发包 (0x0301, 子内容0x0121)" << std::endl;
    std::cout << "  3. 输入自定义十六进制数据包发送" << std::endl;
    std::cout << "  0. 退出" << std::endl;
    std::cout << "请输入选项: ";

    int choice = 0;
    std::cin >> choice;
    std::cin.ignore();

    if (choice == 0) return;

    if (!ser.isOpen())
    {
        std::cerr << "[manual_debug_send] 串口未打开，无法发送" << std::endl;
        return;
    }

    if (choice == 1)
    {
        std::unordered_map<std::string, std::pair<float, float>> test_map;
        std::cout << "输入测试坐标 (格式: 机器人名 x y, 输入 done 结束):" << std::endl;
        std::string name;
        float x, y;
        while (true)
        {
            std::cout << "> ";
            std::cin >> name;
            if (name == "done") break;
            std::cin >> x >> y;
            test_map[name] = {x, y};
        }

        auto ser_data = build_data_radar_all(test_map, color_);
        auto [packet, next_seq] = build_send_packet(ser_data, {0x03, 0x05}, seq_);
        seq_ = next_seq;

        std::cout << "\n[手动发送] 数据域大小: " << ser_data.size() << " bytes" << std::endl;
        std::cout << "[手动发送] 总包大小: " << packet.size() << " bytes" << std::endl;
        std::cout << "[手动发送] 完整数据包(hex): ";
        for (auto byte : packet) std::cout << std::hex << static_cast<int>(byte) << " ";
        std::cout << std::dec << std::endl;

        log_packet_json("serial_tx_log.json", packet, "tx");
        size_t written = ser.write(packet);
        std::cout << "[手动发送] 已发送 " << written << " bytes" << std::endl;
    }
    else if (choice == 2)
    {
        std::cout << "输入 chances_flag 值 (0-255): ";
        int flag_val;
        std::cin >> flag_val;

        auto data = build_data_decision(static_cast<uint8_t>(flag_val), color_);
        auto [packet, next_seq] = build_send_packet(data, {0x03, 0x01}, seq_);
        seq_ = next_seq;

        std::cout << "\n[手动发送] 数据域大小: " << data.size() << " bytes" << std::endl;
        std::cout << "[手动发送] 总包大小: " << packet.size() << " bytes" << std::endl;
        std::cout << "[手动发送] 完整触发包(hex): ";
        for (auto byte : packet) std::cout << std::hex << static_cast<int>(byte) << " ";
        std::cout << std::dec << std::endl;

        log_packet_json("serial_tx_log.json", packet, "tx");
        size_t written = ser.write(packet);
        std::cout << "[手动发送] 已发送 " << written << " bytes" << std::endl;
    }
    else if (choice == 3)
    {
        std::cout << "输入十六进制字节 (空格分隔, 例如: A5 30 00 00 ...):" << std::endl;
        std::cout << "> ";
        std::cin.ignore();
        std::string line;
        std::getline(std::cin, line);

        std::vector<uint8_t> custom_packet;
        std::istringstream iss(line);
        std::string hex_byte;
        while (iss >> hex_byte)
        {
            try
            {
                custom_packet.push_back(static_cast<uint8_t>(std::stoi(hex_byte, nullptr, 16)));
            }
            catch (...)
            {
                std::cerr << "无效的十六进制值: " << hex_byte << std::endl;
            }
        }

        if (custom_packet.empty())
        {
            std::cerr << "[manual_debug_send] 空数据包，取消发送" << std::endl;
            return;
        }

        std::cout << "\n[手动发送] 自定义数据包大小: " << custom_packet.size() << " bytes" << std::endl;
        std::cout << "[手动发送] 数据包内容(hex): ";
        for (auto byte : custom_packet) std::cout << std::hex << static_cast<int>(byte) << " ";
        std::cout << std::dec << std::endl;

        log_packet_json("serial_tx_log.json", custom_packet, "tx");
        size_t written = ser.write(custom_packet);
        std::cout << "[手动发送] 已发送 " << written << " bytes" << std::endl;
    }

    std::cout << "========================================\n" << std::endl;
}

std::vector<uint8_t> SerialManager::serial_read_all()
{
    std::vector<uint8_t> out;
    
    try
    {
        if (!ser.isOpen()) return out;
        size_t avail = ser.available();
        if (avail == 0) return out;
        out.reserve(avail);
        std::string s = ser.read(avail);
        out.assign(s.begin(), s.end());
    }
    catch (const std::exception& e)
    {
        std::cerr << "[serial_read_all] 异常: " << e.what() << std::endl;
    }
    return out;
}

