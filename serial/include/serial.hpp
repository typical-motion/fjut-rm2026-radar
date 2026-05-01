#pragma once

#include <cstdint>
#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <tuple>
#include <mutex>
#include <chrono>
#include <serial/serial.h>

struct RadarDecision
{
    uint8_t double_vulnerability_chance;
    uint8_t opponent_double_vulnerability;
    uint8_t reserved_bits;
};

// Global state variables (defined in serial.cpp)
extern int double_vulnerability_chance;
extern int opponent_double_vulnerability;
extern int chances_flag;
extern std::vector<int> progress_list;
extern std::map<std::string,int> mark_value;
extern std::map<std::string,int> mapping_table;

// CRC constants and tables (defined in serial.cpp)
extern const uint8_t CRC8_INIT;
extern const uint8_t CRC8_TAB[];    // definition in .cpp holds the initializer
extern const uint16_t CRC_INIT;
extern const uint16_t wCRC_Table[]; // definition in .cpp holds the initializer

class SerialPort
{
public:
    SerialPort() = default;
    virtual ~SerialPort() = default;

    // CRC helpers
    uint8_t Get_CRC8_Check_Sum(const std::vector<uint8_t>& pchMessage, size_t dwLength);
    uint16_t Get_CRC16_Check_Sum(const std::vector<uint8_t>& pchMessage, size_t dwLength);

    // Packet parsing / building
    // Returns tuple: (cmd_id_bytes, data_field, seq)
    std::tuple<std::vector<uint8_t>, std::vector<uint8_t>, uint8_t>
    receive_packet(const std::vector<uint8_t>& data, const std::vector<uint8_t>& cmd_id, bool info = false);

    RadarDecision radar_decision(uint8_t data);

    // helpers for appending little-endian uint16
    void append_uint16_t_le(std::vector<uint8_t>& data, uint16_t value);
    void append_uint16_le(std::vector<uint8_t>& data, uint16_t value);

    // build payloads and packets
    std::vector<uint8_t> build_data_radar_all(const std::unordered_map<std::string, std::pair<float,float>>& send_map, char color);
    std::pair<std::vector<uint8_t>, uint8_t> build_send_packet(const std::vector<uint8_t>& data, const std::vector<uint8_t>& cmd_id, uint8_t& seq);
    std::vector<uint8_t> build_data_decision(uint8_t chances, char &color);

protected:
    // protected so derived classes can override / use
    // (note: serial IO object is private in SerialManager)
    // nothing here for now

private:
    // private utility members (if any) can be added here
};


class SerialManager : public SerialPort
{
public:
    // ctor: port name, baudrate (default 115200), color ('R' or 'B')
    SerialManager(const std::string& port, int bandrate = 115200, char color = 'R');
    ~SerialManager();


    // configure and open serial port
    bool serial_set();

    // long-running receive thread loop (reads from serial and updates globals)
    void receive_serial();

    // send periodic and position data to serial; position can be null
    void send_serial(const std::unordered_map<std::string, std::pair<float,float>>* position = nullptr);

    // stop and close serial
    void stop();

private:
    // serial object
    serial::Serial ser;

    std::string port_;
    int bandrate_;
    char color_;
    uint8_t seq_;
    std::chrono::steady_clock::time_point last_send_time_;
    std::mutex mutex_;

    // low-level read helper used by receive_serial (implemented in the .cpp)
    std::vector<uint8_t> serial_read_all();
};