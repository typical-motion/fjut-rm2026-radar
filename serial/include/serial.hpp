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
    uint8_t double_vulnerability_chance;
    uint8_t opponent_double_vulnerability;
    uint8_t reserved_bits;
};

// Global state variables
extern int double_vulnerability_chance;
extern int opponent_double_vulnerability;
extern int chances_flag;
extern std::vector<int> progress_list;
extern std::map<std::string,int> mark_value;
extern std::map<std::string,int> mapping_table;

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

    // Returns tuple: (cmd_id_bytes, data_field, seq)
    std::tuple<std::vector<uint8_t>, std::vector<uint8_t>, uint8_t>
    receive_packet(const std::vector<uint8_t>& data, const std::vector<uint8_t>& cmd_id, bool info = false);

    RadarDecision radar_decision(uint8_t data);

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
    SerialManager(const std::string& port, int bandrate = 115200, char color = 'R');
    ~SerialManager();

    bool serial_set();
    void receive_serial();
    void send_serial(const std::unordered_map<std::string, std::pair<float,float>>* position = nullptr);
    void stop();

private:
    serial::Serial ser;

    std::string port_;
    int bandrate_;
    char color_;
    uint8_t seq_;
    std::chrono::steady_clock::time_point last_send_time_;
    std::mutex mutex_;
    std::atomic<bool> running_{true};

    std::vector<uint8_t> serial_read_all();
};
