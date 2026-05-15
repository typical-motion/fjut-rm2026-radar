#include "MapVisualizer_test.h"
#include <ctime>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>

MapVisualizer::MapVisualizer(const std::string& map_path, int history_length)
    : map_path_(map_path),
      history_length_(history_length),
      map_size_(5.0f, 5.0f),  // 修改为5m x 5m场地
      overlay_alpha_(0.7f)
{
    map_img_ = loadMapImage();
    if (!map_img_.empty()) {
        original_map_ = map_img_.clone();
        map_height_ = map_img_.rows;
        map_width_ = map_img_.cols;
    } else {
        // 创建默认的5m场地背景图
        original_map_ = cv::Mat(500, 500, CV_8UC3, cv::Scalar(30, 30, 30));  // 深灰色背景
        map_height_ = 500;
        map_width_ = 500;
        
        // 绘制场地标记
        drawFieldMarkings(original_map_);
    }

    colors_["B"] = cv::Scalar(255, 140, 0);   // 蓝色方（橙色）
    colors_["R"] = cv::Scalar(0, 0, 255);     // 红色方

    last_update_time_ = std::time(nullptr);

    cv::namedWindow("Radar Map", cv::WINDOW_NORMAL);
    cv::resizeWindow("Radar Map", 500, 500);  // 调整为正方形窗口
    current_map_frame_ = cv::Mat();
}

// 添加绘制场地标记的方法
void MapVisualizer::drawFieldMarkings(cv::Mat& img) {
    // 绘制中心点
    cv::Point center = worldToPixel({2.5f, 2.5f});
    cv::circle(img, center, 10, cv::Scalar(255, 255, 255), 2);
    
    // 绘制中线
    cv::line(img, 
             worldToPixel({2.5f, 0.0f}),
             worldToPixel({2.5f, 5.0f}),
             cv::Scalar(200, 200, 200), 2);
    
    // 绘制基地标记
    // 自方基地（左下）
    cv::rectangle(img,
                  worldToPixel({0.2f, 0.2f}),
                  worldToPixel({0.8f, 0.8f}),
                  cv::Scalar(255, 140, 0), 2);
    
    // 敌方基地（右上）
    cv::rectangle(img,
                  worldToPixel({4.2f, 4.2f}),
                  worldToPixel({4.8f, 4.8f}),
                  cv::Scalar(0, 0, 255), 2);
    
    // 绘制文字标记
    cv::putText(img, "Blue Base", worldToPixel({0.2f, 1.0f}),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 140, 0), 1);
    cv::putText(img, "Red Base", worldToPixel({3.5f, 4.5f}),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
}

cv::Mat MapVisualizer::loadMapImage() {
    cv::Mat img = cv::imread(map_path_);
    if (img.empty()) {
        std::cerr << "警告: 地图图像未找到或无法加载 '" << map_path_ << "'\n";
        std::cerr << "使用默认的5m场地模板\n";
        return cv::Mat(500, 500, CV_8UC3, cv::Scalar(30, 30, 30));
    }
    return img;
}

cv::Point MapVisualizer::worldToPixel(const cv::Point2f& world_pos) const {
    // 5m场地的坐标转换
    float x_scale = map_width_ / map_size_.first;   // 500/5 = 100 pixels/m
    float y_scale = map_height_ / map_size_.second; // 500/5 = 100 pixels/m
    
    // 确保坐标在场地范围内
    float clamped_x = std::max(0.0f, std::min(world_pos.x, map_size_.first));
    float clamped_y = std::max(0.0f, std::min(world_pos.y, map_size_.second));
    
    int pixel_x = static_cast<int>(clamped_x * x_scale);
    int pixel_y = static_cast<int>(map_height_ - clamped_y * y_scale); // Y轴翻转
    
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
        cv::line(map_display, points[i-1], points[i], color, std::max(1, int(2*alpha)), cv::LINE_AA);
    }
}

cv::Mat MapVisualizer::updateMap(const std::map<std::string, cv::Point2f>& enemy_positions,
                                 const std::map<std::string, cv::Point2f>* friendly_positions)
{
    cv::Mat map_display = original_map_.clone();
    overlay_alpha_ = 0.7f;
    double current_time = std::time(nullptr);
    cv::Mat overlay = map_display.clone();

    // 绘制网格线（1m间隔）
    float grid_spacing = 1.0f;
    cv::Scalar grid_color(100, 100, 100);
    int grid_thickness = 1;

    for (int y = 0; y <= static_cast<int>(map_size_.second); y += static_cast<int>(grid_spacing)) {
        cv::line(map_display, 
                worldToPixel({0.0f, static_cast<float>(y)}), 
                worldToPixel({static_cast<float>(map_size_.first), static_cast<float>(y)}), 
                grid_color, grid_thickness, cv::LINE_AA);
    }
    for (int x = 0; x <= static_cast<int>(map_size_.first); x += static_cast<int>(grid_spacing)) {
        cv::line(map_display, 
                worldToPixel({static_cast<float>(x), 0.0f}), 
                worldToPixel({static_cast<float>(x), static_cast<float>(map_size_.second)}), 
                grid_color, grid_thickness, cv::LINE_AA);
    }

    // 绘制半米辅助线（更淡的线）
    float half_grid_spacing = 0.5f;
    cv::Scalar half_grid_color(60, 60, 60);
    
    for (float y = half_grid_spacing; y < map_size_.second; y += grid_spacing) {
        cv::line(map_display, 
                worldToPixel({0.0f, y}), 
                worldToPixel({static_cast<float>(map_size_.first), y}), 
                half_grid_color, 1, cv::LINE_AA);
    }
    for (float x = half_grid_spacing; x < map_size_.first; x += grid_spacing) {
        cv::line(map_display, 
                worldToPixel({x, 0.0f}), 
                worldToPixel({x, static_cast<float>(map_size_.second)}), 
                half_grid_color, 1, cv::LINE_AA);
    }

    // 绘制敌人位置（圆形）
    for (const auto& [enemy_id, position] : enemy_positions) {
        char team = enemy_id[0];
        cv::Scalar color = colors_.count(std::string(1, team)) ? colors_.at(std::string(1, team)) : cv::Scalar(0,255,0);
        
        // 确保坐标在场地内
        cv::Point2f clamped_pos = clampToField(position);
        updatePositionHistory(enemy_id, clamped_pos);
        cv::Point pixel_pos = worldToPixel(clamped_pos);
        drawPositionHistory(map_display, enemy_id);

        // 敌人用实心圆表示
        cv::circle(overlay, pixel_pos, 12, color, -1);
        cv::circle(map_display, pixel_pos, 8, color, -1);
        cv::circle(map_display, pixel_pos, 10, cv::Scalar(255,255,255), 2);

        // 显示ID
        cv::putText(map_display, enemy_id, pixel_pos + cv::Point(15, -5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 2);
        cv::putText(map_display, enemy_id, pixel_pos + cv::Point(15, -5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);

        last_positions_[enemy_id] = clamped_pos;
    }

    // 绘制友方位置（三角形）
    if (friendly_positions) {
        for (const auto& [friendly_id, position] : *friendly_positions) {
            char team = friendly_id[0];
            cv::Scalar color = colors_.count(std::string(1, team)) ? colors_.at(std::string(1, team)) : cv::Scalar(0,255,0);

            // 确保坐标在场地内
            cv::Point2f clamped_pos = clampToField(position);
            updatePositionHistory(friendly_id, clamped_pos);
            cv::Point pixel_pos = worldToPixel(clamped_pos);
            drawPositionHistory(map_display, friendly_id);

            // 友方用三角形表示
            std::vector<cv::Point> triangle_points = {
                {pixel_pos.x, pixel_pos.y-10},
                {pixel_pos.x-7, pixel_pos.y+6},
                {pixel_pos.x+7, pixel_pos.y+6}
            };
            std::vector<std::vector<cv::Point>> triangles = {triangle_points};
            cv::fillPoly(overlay, triangles, color);
            cv::fillPoly(map_display, triangles, color);
            cv::polylines(map_display, triangles, true, cv::Scalar(255,255,255), 2);

            // 显示ID
            cv::putText(map_display, friendly_id, pixel_pos + cv::Point(15, -5), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 2);
            cv::putText(map_display, friendly_id, pixel_pos + cv::Point(15, -5), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);

            last_positions_[friendly_id] = clamped_pos;
        }
    }

    // 混合叠加层
    cv::addWeighted(overlay, 0.3, map_display, 0.7, 0, map_display);

    // 时间戳
    std::time_t t = std::time(nullptr);
    char time_buff[16];
    std::strftime(time_buff, sizeof(time_buff), "%H:%M:%S", std::localtime(&t));
    cv::putText(map_display, std::string("Time: ") + time_buff, 
                cv::Point(10, map_height_ - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);

    // 比例尺（1米 = 100像素）
    cv::Point scale_start(map_width_ - 120, map_height_ - 30);
    cv::Point scale_end  (map_width_ - 20,  map_height_ - 30);
    cv::line(map_display, scale_start, scale_end, cv::Scalar(255,255,255), 2);
    cv::putText(map_display, "1m", scale_start + cv::Point(30, -5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);

    // 添加坐标轴标签
    cv::putText(map_display, "X", worldToPixel({4.8f, 0.2f}),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200,200,200), 1);
    cv::putText(map_display, "Y", worldToPixel({0.2f, 4.8f}),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200,200,200), 1);

    last_update_time_ = current_time;
    current_map_frame_ = map_display.clone();
    return map_display;
}

// 添加坐标限制函数
cv::Point2f MapVisualizer::clampToField(const cv::Point2f& pos) const {
    return cv::Point2f(
        std::max(0.0f, std::min(pos.x, map_size_.first)),
        std::max(0.0f, std::min(pos.y, map_size_.second))
    );
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
    position_history_.clear();
    last_positions_.clear();
}

void MapVisualizer::addEnemy(const std::string& robot_id, float x, float y) {
    // 对于5m场地，坐标范围是0-5，不需要Y轴反向
    current_enemy_positions_[robot_id] = cv::Point2f(x, y);
}

void MapVisualizer::addFriendly(const std::string& robot_id, float x, float y) {
    current_friendly_positions_[robot_id] = cv::Point2f(x, y);
}

void MapVisualizer::update() {
    cv::Mat map_display = updateMap(current_enemy_positions_, &current_friendly_positions_);
    cv::imshow("Radar Map", map_display);
    cv::waitKey(1);
}

cv::Mat MapVisualizer::getMapFrame() {
    if (current_map_frame_.empty()) {
        cv::Mat map_display = original_map_.clone();

        // 绘制网格
        float grid_spacing = 1.0f;
        cv::Scalar grid_color(100, 100, 100);

        for (int y = 0; y <= static_cast<int>(map_size_.second); y += static_cast<int>(grid_spacing)) {
            cv::line(map_display, 
                worldToPixel({0.0f, static_cast<float>(y)}), 
                worldToPixel({static_cast<float>(map_size_.first), static_cast<float>(y)}), 
                grid_color, 1, cv::LINE_AA);
        }
        for (int x = 0; x <= static_cast<int>(map_size_.first); x += static_cast<int>(grid_spacing)) {
            cv::line(map_display, 
                worldToPixel({static_cast<float>(x), 0.0f}), 
                worldToPixel({static_cast<float>(x), static_cast<float>(map_size_.second)}), 
                grid_color, 1, cv::LINE_AA);
        }

        // 时间戳
        std::time_t t = std::time(nullptr);
        char time_buff[16];
        std::strftime(time_buff, sizeof(time_buff), "%H:%M:%S", std::localtime(&t));
        cv::putText(map_display, std::string("Time: ") + time_buff, 
                    cv::Point(10, map_height_ - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);

        // 比例尺
        cv::Point scale_start(map_width_ - 120, map_height_ - 30);
        cv::Point scale_end  (map_width_ - 20,  map_height_ - 30);
        cv::line(map_display, scale_start, scale_end, cv::Scalar(255,255,255), 2);
        cv::putText(map_display, "1m", scale_start + cv::Point(30, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);

        current_map_frame_ = map_display.clone();
    }
    return current_map_frame_;
}

cv::Size MapVisualizer::getMapSize() const {
    return cv::Size(map_width_, map_height_);
}