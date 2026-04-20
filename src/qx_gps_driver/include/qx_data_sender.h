//
// Created by remotelink on 25-6-3.
//

#ifndef DATASENDER_H
#define DATASENDER_H

#include <thread>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include "qx_data_parser.h"

class DataSender {
public:
    DataSender(ros::NodeHandle& node);
    ~DataSender() = default;

    bool run();

    void close();

    std::thread sendThread;

private:
    // 读取配置文件
    void loadConfig();
    // 根据杆臂值计算偏移量
    void calib(const double &heading, const double &lever_arm, double &x, double &y);
    // 经纬度转UTM
    void LL2UTM(const double &lat, const double &lon, double &x, double &y, int &zone, bool &northp);
    // UTM转经纬度
    void UTM2LL(const double &x, const double &y, const int& zone, const bool &northp, double &lat, double &lon);

    ros::NodeHandle nh;
    ros::Publisher pub;         // sensor_msgs/NavSatFix
    ros::Publisher pub_vel;     // geometry_msgs/TwistStamped (ENU速度)
    ros::Publisher pub_heading; // std_msgs/Float64 (航向角°)

    std::shared_ptr<DataParser> data_parser_ptr;

    std::string topic;          // NavSatFix 话题名
    std::string topic_vel;      // 速度话题名
    std::string topic_heading;  // 航向话题名
    std::string port;           // 串口名
    unsigned int baud_rate; // 波特率
    double lever_arm;       // 杆臂值
    double offset_angle;    // 航向偏移角

    bool exit_flag = true;

    std::ofstream fout_debug;

};



#endif //DATASENDER_H
