#include "MapVisualizer.h"
#include <ctime>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>

MapVisualizer::MapVisualizer(const std::string& map_path, int history_length)
    : map_path_(map_path),
      history_length_(history_length),
      map_size_(28.0f, 15.0f),
      overlay_alpha_(0.7f)
{
    map_img_ = loadMapImage();
    if (!map_img_.empty()) {
        original_map_ = map_img_.clone();
        map_height_ = map_img_.rows;
        map_width_ = map_img_.cols;
    } else {
        original_map_ = cv::Mat(800, 1500, CV_8UC3, cv::Scalar(0,0,0));
        map_height_ = 800;
        map_width_ = 1500;
    }

    colors_["B"] = cv::Scalar(255, 140, 0);
    colors_["R"] = cv::Scalar(0, 0, 255);

    last_update_time_ = std::time(nullptr);

    cv::namedWindow("Radar Map", cv::WINDOW_NORMAL);
    cv::resizeWindow("Radar Map", 800, 450);
    current_map_frame_ = cv::Mat();
}

cv::Mat MapVisualizer::loadMapImage() {
    cv::Mat img = cv::imread(map_path_);
    if (img.empty()) {
        std::cerr << "警告: 地图图像未找到或无法加载 '" << map_path_ << "'\n";
        return cv::Mat(800, 1500, CV_8UC3, cv::Scalar(0,0,0));
    }
    return img;
}

cv::Point MapVisualizer::worldToPixel(const cv::Point2f& world_pos) const {
    float x_scale = map_width_ / map_size_.first;
    float y_scale = map_height_ / map_size_.second;
    int pixel_x = static_cast<int>(world_pos.x * x_scale);
    int pixel_y = static_cast<int>(map_height_ - world_pos.y * y_scale);
    return cv::Point(pixel_x, pixel_y);
}

void MapVisualizer::updatePositionHistory(const std::string& robot_id, const cv::Point2f& position) {
    if (position_history_.find(robot_id) == position_history_.end()) {
        position_history_[robot_id] = std::deque<cv::Point>();
        position_history_[robot_id].resize(0);
    }
    cv::Point pixel_pos = worldToPixel(position);
    if (position_history_[robot_id].size() >= static_cast<size_t>(history_length_))
        position_history_[robot_id].pop_front();
    position_history_[robot_id].push_back(pixel_pos);
}

void MapVisualizer::drawPositionHistory(cv::Mat& map_display, const std::string& robot_id) const {
    auto it = position_history_.find(robot_id);
    if (it == position_history_.end() || it->second.size() < 2) return;

    char team = robot_id[0];
    cv::Scalar color = colors_.count(std::string(1, team)) ? colors_.at(std::string(1, team)) : cv::Scalar(0,255,0);

    const auto& points = it->second;
    for (size_t i = 1; i < points.size(); ++i) {
        double alpha = 0.3 + 0.7 * (static_cast<double>(i) / points.size());
        cv::line(map_display, points[i-1], points[i], color, std::max(1, int(3*alpha)), cv::LINE_AA);
    }
}

cv::Mat MapVisualizer::updateMap(const std::map<std::string, cv::Point2f>& enemy_positions,
                                 const std::map<std::string, cv::Point2f>* friendly_positions)
{
    cv::Mat map_display = original_map_.clone();
    overlay_alpha_ = 0.7f;
    double current_time = std::time(nullptr);
    cv::Mat overlay = map_display.clone();

    float grid_spacing = 5.0f;
    cv::Scalar grid_color(150, 150, 150);

    for (int y = 0; y <= static_cast<int>(map_size_.second); y += static_cast<int>(grid_spacing)) {
        cv::line(map_display, 
                worldToPixel({0.0f,static_cast<float>(y)}), 
                worldToPixel({static_cast<float>(map_size_.first), static_cast<float>(y)}), 
                grid_color, 1, cv::LINE_AA);
    }
    for (int x = 0; x <= static_cast<int>(map_size_.first); x += static_cast<int>(grid_spacing)) {
        cv::line(map_display, worldToPixel({(float)x,0}), worldToPixel({(float)x,map_size_.second}), grid_color, 1, cv::LINE_AA);
    }

    for (const auto& [enemy_id, position] : enemy_positions) {
        char team = enemy_id[0];
        cv::Scalar color = colors_.count(std::string(1, team)) ? colors_.at(std::string(1, team)) : cv::Scalar(0,255,0);
        updatePositionHistory(enemy_id, position);
        cv::Point pixel_pos = worldToPixel(position);
        drawPositionHistory(map_display, enemy_id);

        cv::circle(overlay, pixel_pos, 15, color, -1);
        cv::circle(map_display, pixel_pos, 10, color, -1);
        cv::circle(map_display, pixel_pos, 12, color, 2);

        cv::putText(map_display, enemy_id, pixel_pos + cv::Point(15,5), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 5);
        cv::putText(map_display, enemy_id, pixel_pos + cv::Point(15,5), cv::FONT_HERSHEY_SIMPLEX, 2, color, 5);

        last_positions_[enemy_id] = position;
    }

    if (friendly_positions) {
        for (const auto& [friendly_id, position] : *friendly_positions) {
            char team = friendly_id[0];
            cv::Scalar color = colors_.count(std::string(1, team)) ? colors_.at(std::string(1, team)) : cv::Scalar(0,255,0);

            updatePositionHistory(friendly_id, position);
            cv::Point pixel_pos = worldToPixel(position);
            drawPositionHistory(map_display, friendly_id);

            std::vector<cv::Point> triangle_points = {
                {pixel_pos.x, pixel_pos.y-12},
                {pixel_pos.x-8, pixel_pos.y+6},
                {pixel_pos.x+8, pixel_pos.y+6}
            };
            std::vector<std::vector<cv::Point>> triangles = {triangle_points};
            cv::fillPoly(overlay, triangles, color);
            cv::fillPoly(map_display, triangles, color);
            cv::polylines(map_display, triangles, true, cv::Scalar(255,255,255), 2);

            cv::putText(map_display, friendly_id, pixel_pos + cv::Point(15,5), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 5);
            cv::putText(map_display, friendly_id, pixel_pos + cv::Point(15,5), cv::FONT_HERSHEY_SIMPLEX, 2, color, 5);

            last_positions_[friendly_id] = position;
        }
    }

    cv::addWeighted(overlay, 0.3, map_display, 0.7, 0, map_display);

    // 时间戳
    std::time_t t = std::time(nullptr);
    char time_buff[16];
    std::strftime(time_buff, sizeof(time_buff), "%H:%M:%S", std::localtime(&t));
    cv::putText(map_display, std::string("Time: ") + time_buff, cv::Point(10, map_height_ - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 2);

    // 比例尺
    cv::Point scale_start(map_width_ - 120, map_height_ - 30);
    cv::Point scale_end  (map_width_ - 20,  map_height_ - 30);
    cv::line(map_display, scale_start, scale_end, cv::Scalar(255,255,255), 2);

    last_update_time_ = current_time;
    current_map_frame_ = map_display.clone();
    return map_display;
}

void MapVisualizer::showMap(const std::map<std::string, cv::Point2f>& enemy_positions,
                            const std::map<std::string, cv::Point2f>* friendly_positions,
                            const std::string& window_name)
{
    cv::Mat map_display = updateMap(enemy_positions, friendly_positions);
    cv::imshow(window_name, map_display);
}

void MapVisualizer::clear() {
    current_enemy_positions_.clear();
    current_friendly_positions_.clear();
}

void MapVisualizer::addEnemy(const std::string& robot_id, float x, float y) {
    // y轴反向
    current_enemy_positions_[robot_id] = cv::Point2f(x, 15.0f - y);
}

void MapVisualizer::addFriendly(const std::string& robot_id, float x, float y) {
    current_friendly_positions_[robot_id] = cv::Point2f(x, 15.0f - y);
}

void MapVisualizer::update() {
    cv::Mat map_display = updateMap(current_enemy_positions_, &current_friendly_positions_);
    cv::imshow("Radar Map", map_display);
    cv::waitKey(1);
}

cv::Mat MapVisualizer::getMapFrame() {
    if (current_map_frame_.empty()) {
        cv::Mat map_display = original_map_.clone();

        float grid_spacing = 5.0f;
        cv::Scalar grid_color(150, 150, 150);

        for (int y = 0; y <= static_cast<int>(map_size_.second); y += static_cast<int>(grid_spacing)) {
            cv::line(map_display, 
                worldToPixel({0.0f,static_cast<float>(y)}), 
                worldToPixel({static_cast<float>(map_size_.first), static_cast<float>(y)}), 
                grid_color, 1, cv::LINE_AA);
        }
        for (int x = 0; x <= static_cast<int>(map_size_.first); x += static_cast<int>(grid_spacing)) {
            cv::line(map_display, worldToPixel({(float)x,0}), worldToPixel({(float)x,map_size_.second}), grid_color, 1, cv::LINE_AA);
        }

        std::time_t t = std::time(nullptr);
        char time_buff[16];
        std::strftime(time_buff, sizeof(time_buff), "%H:%M:%S", std::localtime(&t));
        cv::putText(map_display, std::string("Time: ") + time_buff, cv::Point(10, map_height_ - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 2);

        cv::Point scale_start(map_width_ - 120, map_height_ - 30);
        cv::Point scale_end  (map_width_ - 20,  map_height_ - 30);
        cv::line(map_display, scale_start, scale_end, cv::Scalar(255,255,255), 2);

        current_map_frame_ = map_display.clone();
    }
    return current_map_frame_;
}

cv::Size MapVisualizer::getMapSize() const {
    return cv::Size(map_width_, map_height_);
}