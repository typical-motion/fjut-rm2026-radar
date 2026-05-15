#pragma once

#include <cstdint>
#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <tuple>
#include <mutex>
#include <atomic>
#include <chrono>
#include <serial/serial.h>

struct RadarDecision
{
    uint8_t double_vulnerability_chance;  // bit 0-1: 双倍易伤触发机会计数 (0-2)
    uint8_t opponent_double_vulnerability; // bit 2: 对方是否正在被触发双倍易伤
    uint8_t encryption_level;             // bit 3-4: 己方加密等级 (1-3)
    uint8_t key_modifiable;               // bit 5: 是否可修改密钥
    uint8_t reserved_bits;                // bit 6-7: 保留位
};

// Global state variables
extern int double_vulnerability_chance;
extern int opponent_double_vulnerability;
extern int chances_flag;
extern std::vector<int> progress_list;
extern std::map<std::string,int> mark_value;
extern std::map<std::string,int> mapping_table;
extern int key_staus;
extern int wave_key_received;
extern int encryption_level;
extern int key_modifiable;
extern int game_type;
extern int game_progress;
extern int stage_remain_time;
extern uint64_t sync_timestamp;

// CRC constants and tables
extern const uint8_t CRC8_INIT;
extern const uint8_t CRC8_TAB[];
extern const uint16_t CRC_INIT;
extern const uint16_t wCRC_Table[];

class SerialPort
{
public:
    SerialPort() = default;
    virtual ~SerialPort() = default;

    // CRC helpers
    uint8_t Get_CRC8_Check_Sum(const std::vector<uint8_t>& pchMessage, size_t dwLength);
    uint16_t Get_CRC16_Check_Sum(const std::vector<uint8_t>& pchMessage, size_t dwLength);

    // Returns tuple: (cmd_id_bytes, data_field, seq) — 需指定 cmd_id 匹配
    std::tuple<std::vector<uint8_t>, std::vector<uint8_t>, uint8_t>
    receive_packet(const std::vector<uint8_t>& data, const std::vector<uint8_t>& cmd_id, bool info = false);

    // 通用提取: 不筛选 cmd_id，直接返回完整包的 (cmd_id_bytes, data_field, seq)
    std::tuple<std::vector<uint8_t>, std::vector<uint8_t>, uint8_t>
    receive_any_packet(const std::vector<uint8_t>& data, bool info = false);

    RadarDecision radar_decision(uint8_t data);

    // 打印接收到的数据包内容
    static void print_received_data(const std::vector<uint8_t>& cmd_id_bytes,
                                     const std::vector<uint8_t>& data_field,
                                     uint8_t seq);

    // Little-endian uint16 append (t_le alias kept for compatibility)
    void append_uint16_t_le(std::vector<uint8_t>& data, uint16_t value);
    void append_uint16_le(std::vector<uint8_t>& data, uint16_t value);

    // Build payloads and packets
    std::vector<uint8_t> build_data_radar_all(const std::unordered_map<std::string, std::pair<float,float>>& send_map, char color);
    std::pair<std::vector<uint8_t>, uint8_t> build_send_packet(const std::vector<uint8_t>& data, const std::vector<uint8_t>& cmd_id, uint8_t& seq);
    std::vector<uint8_t> build_data_decision(uint8_t chances, char color);
};


class SerialManager : public SerialPort
{
public:
    SerialManager(const std::string& port, int bandrate = 115200, char color = 'R', bool debug = false);
    ~SerialManager();

    bool serial_set();
    void receive_serial();
    void send_serial(const std::unordered_map<std::string, std::pair<float,float>>* position = nullptr);
    void send_serial_key(uint8_t /*id1*/, uint8_t id2, const std::string& key_str);
    void stop();
    void set_debug_mode(bool mode);
    void manual_debug_send();

private:
    serial::Serial ser;

    std::string port_;
    int bandrate_;
    char color_;
    uint8_t seq_;
    bool debug_mode_;
    std::chrono::steady_clock::time_point last_send_time_;
    std::chrono::steady_clock::time_point trigger_window_start_;
    std::mutex mutex_;
    std::atomic<bool> running_{true};
    int trigger_count_ = 0;
    bool trigger_active_ = false;

    // Position cache: retains last known coordinates when not updated
    std::unordered_map<std::string, std::pair<float,float>> pos_cache_;

    std::vector<uint8_t> serial_read_all();
};
