#pragma once
#include <opencv2/opencv.hpp>
#include <map>
#include <string>
#include <deque>

class MapVisualizer {
public:
    MapVisualizer(const std::string& map_path, int history_length = 20);
    
    void addEnemy(const std::string& robot_id, float x, float y);
    void addFriendly(const std::string& robot_id, float x, float y);
    void update();
    void clear();
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
                      const std::map<std::string, cv::Point2f>* friendly_positions);
    cv::Point2f clampToField(const cv::Point2f& pos) const;
    void drawFieldMarkings(cv::Mat& img);

    std::string map_path_;
    cv::Mat map_img_;
    cv::Mat original_map_;
    int map_width_;
    int map_height_;
    std::pair<float, float> map_size_;  // 场地大小 (width, height)
    float overlay_alpha_;
    
    std::map<std::string, cv::Scalar> colors_;
    std::map<std::string, cv::Point2f> last_positions_;
    std::map<std::string, std::deque<cv::Point>> position_history_;
    int history_length_;
    
    double last_update_time_;
    
    std::map<std::string, cv::Point2f> current_enemy_positions_;
    std::map<std::string, cv::Point2f> current_friendly_positions_;
    cv::Mat current_map_frame_;
};