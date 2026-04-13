#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <deque>
#include <map>

class MapVisualizer {
public:
    MapVisualizer(const std::string& map_path = "/home/zqz/ros2_ws/image/map.jpg", int history_length = 10);

    void clear();
    void addEnemy(const std::string& robot_id, float x, float y);
    void addFriendly(const std::string& robot_id, float x, float y);
    void update();
    void showMap(const std::map<std::string, cv::Point2f>& enemy_positions,
                const std::map<std::string, cv::Point2f>* friendly_positions = nullptr,
                const std::string& window_name = "Radar Map");
    cv::Mat getMapFrame();
    cv::Size getMapSize() const;

private:
    cv::Mat loadMapImage();
    cv::Point worldToPixel(const cv::Point2f& world_pos) const;
    void updatePositionHistory(const std::string& robot_id, const cv::Point2f& position);
    void drawPositionHistory(cv::Mat& map_display, const std::string& robot_id) const;
    cv::Mat updateMap(const std::map<std::string, cv::Point2f>& enemy_positions,
                      const std::map<std::string, cv::Point2f>* friendly_positions = nullptr);

    std::string map_path_;
    cv::Mat map_img_;
    cv::Mat original_map_;
    int map_width_;
    int map_height_;
    int history_length_;
    std::map<std::string, cv::Scalar> colors_;
    std::map<std::string, std::deque<cv::Point>> position_history_;
    std::pair<float, float> map_size_;

    double last_update_time_;
    std::map<std::string, cv::Point2f> last_positions_;

    float overlay_alpha_;

    std::map<std::string, cv::Point2f> current_enemy_positions_;
    std::map<std::string, cv::Point2f> current_friendly_positions_;

    cv::Mat current_map_frame_;
};