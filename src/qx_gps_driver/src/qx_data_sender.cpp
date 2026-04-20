#include "qx_data_sender.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>  // 引入 UTM 转换头文件

#define DEG2RAD 0.0174532925199433
#define RAD2DEG 57.29577951308232

//Normalizes the angle in radians between[-pi and pi).
static double NormalizeAngle_radian(const double &angle_radians) {
    return angle_radians - (2.0 * M_PI) * std::floor((angle_radians + M_PI) / (2.0 * M_PI));
}

//360°版
static double NormalizeAngle_degree(const double &angle_radians) {
    return angle_radians - (2.0 * 180.0) * std::floor((angle_radians + 180.0) / (2.0 * 180.0));
}


// 将 KSXT pos_qual 映射为标准 NavSatStatus
// KSXT: 0=不可用, 1=单点, 2=伪距差分/SBAS, 4=RTK固定解, 5=RTK浮点解, 6=惯导
// NavSatStatus: STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2
static int8_t posQualToNavSatStatus(unsigned int pos_qual) {
    switch (pos_qual) {
        case 1:  return sensor_msgs::NavSatStatus::STATUS_FIX;
        case 2:  return sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        case 4:  return sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        case 5:  return sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        case 6:  return sensor_msgs::NavSatStatus::STATUS_FIX;
        default: return sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }
}

// 按定位质量返回水平/垂直位置方差（m²），近似值
// 对角协方差矩阵：[east_var, 0, 0, 0, north_var, 0, 0, 0, up_var]
static void fillPositionCovariance(unsigned int pos_qual,
                                   boost::array<double, 9> &cov,
                                   uint8_t &cov_type) {
    double h_var, v_var;
    switch (pos_qual) {
        case 4:  h_var = 0.0004; v_var = 0.0016; break;  // RTK固定解 ±0.02m/±0.04m
        case 5:  h_var = 0.09;   v_var = 0.25;   break;  // RTK浮点解 ±0.3m/±0.5m
        case 6:  h_var = 0.25;   v_var = 1.0;    break;  // 惯导       ±0.5m/±1.0m
        case 2:  h_var = 1.0;    v_var = 4.0;    break;  // SBAS差分  ±1.0m/±2.0m
        case 1:  h_var = 9.0;    v_var = 25.0;   break;  // 单点定位  ±3.0m/±5.0m
        default:
            cov.fill(0.0);
            cov_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
            return;
    }
    cov.fill(0.0);
    cov[0] = h_var;  // east
    cov[4] = h_var;  // north
    cov[8] = v_var;  // up
    cov_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
}

DataSender::DataSender(ros::NodeHandle &node) {
    nh = node;

    loadConfig();
    data_parser_ptr = std::make_shared<DataParser>(port, baud_rate);
    pub         = nh.advertise<sensor_msgs::NavSatFix>(topic, 10);
    pub_vel     = nh.advertise<geometry_msgs::TwistStamped>(topic_vel, 10);
    pub_heading = nh.advertise<std_msgs::Float64>(topic_heading, 10);
}

bool DataSender::run() {
    data_parser_ptr->setCallback([&](const KSXT *msg) {
        const ros::Time now = ros::Time::now();

        // ── 1. sensor_msgs/NavSatFix ──────────────────────────────────────
        sensor_msgs::NavSatFix nav_msg;
        nav_msg.header.stamp    = now;
        nav_msg.header.frame_id = "wgs84";

        nav_msg.altitude = msg->data.height;

        // NavSatStatus 标准枚举映射
        nav_msg.status.status  = posQualToNavSatStatus(msg->data.pos_qual);
        // GPS + BeiDou 位掩码（SERVICE_GPS=1, SERVICE_COMPASS/BDS=4）
        nav_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS |
                                 sensor_msgs::NavSatStatus::SERVICE_COMPASS;

        // 位置协方差（对角阵，按定位质量近似，单位 m²）
        fillPositionCovariance(msg->data.pos_qual,
                               nav_msg.position_covariance,
                               nav_msg.position_covariance_type);

        // 杆臂改正（仅双天线定向有效时）
        if (msg->data.heading_qual == 4) {
            double delta_x, delta_y, x, y, lat, lon;
            int zone; bool northp;
            LL2UTM(msg->data.lat, msg->data.lon, x, y, zone, northp);
            calib(msg->data.heading, lever_arm, delta_x, delta_y);
            x += delta_x; y += delta_y;
            UTM2LL(x, y, zone, northp, lat, lon);
            nav_msg.latitude  = lat;
            nav_msg.longitude = lon;
        } else {
            nav_msg.latitude  = msg->data.lat;
            nav_msg.longitude = msg->data.lon;
        }

        pub.publish(nav_msg);

        // ── 2. geometry_msgs/TwistStamped（ENU 速度，m/s）────────────────
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.header.stamp    = now;
        vel_msg.header.frame_id = "wgs84";
        vel_msg.twist.linear.x  = msg->data.vel_east;   // 东向
        vel_msg.twist.linear.y  = msg->data.vel_north;  // 北向
        vel_msg.twist.linear.z  = msg->data.vel_up;     // 天向

        pub_vel.publish(vel_msg);

        // ── 3. std_msgs/Float64（航向角°，真北为0，顺时针，已叠加安装偏移）
        double heading_temp = msg->data.heading + offset_angle;
        if (heading_temp >= 360.0) heading_temp -= 360.0;
        else if (heading_temp < 0.0) heading_temp += 360.0;

        std_msgs::Float64 heading_msg;
        heading_msg.data = heading_temp;
        pub_heading.publish(heading_msg);
    });
    if (!data_parser_ptr->run_parse())
        return false;
    return true;
}

void DataSender::close() {
    data_parser_ptr->close();
}


void DataSender::loadConfig() {
    nh.param<std::string>("serial_port", port, "/dev/ttyUSB0");
    nh.param<std::string>("topic",         topic,         "/qx/evk");
    nh.param<std::string>("topic_vel",     topic_vel,     "/qx/evk/velocity");
    nh.param<std::string>("topic_heading", topic_heading, "/qx/evk/heading");
    int baud_rate_int;
    nh.param<int>("baud_rate", baud_rate_int, 460800);
    nh.param<double>("lever_arm",     lever_arm,    0.0);
    nh.param<double>("offset_angle",  offset_angle, 0.0);

    ROS_INFO("serial: %s @ %d baud | topic: %s | vel: %s | heading: %s | lever_arm: %.3f m | offset: %.1f deg",
             port.c_str(), baud_rate_int,
             topic.c_str(), topic_vel.c_str(), topic_heading.c_str(),
             lever_arm, offset_angle);
    baud_rate = static_cast<unsigned int>(baud_rate_int);
}

void DataSender::calib(const double &heading, const double &lever_arm, double &delta_x,
                       double &delta_y) {
    double yaw = NormalizeAngle_degree(heading);
    delta_x = lever_arm * cos(M_PI_2 - yaw);
    delta_y = lever_arm * sin(M_PI_2 - yaw);
    // printf("x: %lf, y: %lf\n", x, y);
}

void DataSender::LL2UTM(const double &lat, const double &lon, double &x, double &y, int &zone, bool &northp) {
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y); // 使用 GeographicLib 进行 UTM 转换
}

void DataSender::UTM2LL(const double &x, const double &y, const int &zone, const bool &northp, double &lat,
                        double &lon) {
    GeographicLib::UTMUPS::Reverse(zone, northp, x, y, lat, lon); // 使用 GeographicLib 进行 UTM 转换
}

static bool flag_exit = false;

void signalHandler(int signum) {
    flag_exit = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "qx_data_sender");
    ros::NodeHandle nh;
    DataSender data_sender(nh);

    // 启动数据发送线程
    std::thread sendThread([&data_sender]() {
        if (!data_sender.run()) {
            ROS_ERROR("Failed to start data sender");
        }
    });

    // 等待节点运行
    ros::spin();

    // 关闭数据发送线程
    data_sender.close();
    if (sendThread.joinable()) {
        sendThread.join();
    }

    return 0;
}
