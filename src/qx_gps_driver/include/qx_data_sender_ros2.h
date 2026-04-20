#ifndef DATASENDERROS2_H
#define DATASENDERROS2_H

#include <thread>
#include <string>

#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "qx_data_parser.h"

class DataSenderROS2 {
public:
    DataSenderROS2(rclcpp::Node::SharedPtr node_ptr);
    ~DataSenderROS2() = default;

    bool run();

    void close();

    std::thread sendThread;
    rclcpp::Node::SharedPtr node_ptr;


private:
    // 加载配置参数
    void loadConfig();
    // 根据杆臂值计算偏移量
    void calib(const double &heading, const double &lever_arm, double &x, double &y);
    // 经纬度转UTM
    void LL2UTM(const double &lat, const double &lon, double &x, double &y, int &zone, bool &northp);
    // UTM转经纬度
    void UTM2LL(const double &x, const double &y, const int& zone, const bool &northp, double &lat, double &lon);

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_ptr;
    std::shared_ptr<DataParser> data_parser_ptr;

    std::string topic;
    std::string port;
    unsigned int baud_rate;
    double lever_arm;
    double offset_angle;    // 航向偏移角



    bool exit_flag = true;
};

#endif // DATASENDERROS2_H