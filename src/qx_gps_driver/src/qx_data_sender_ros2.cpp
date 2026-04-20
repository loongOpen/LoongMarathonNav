#include "qx_data_sender_ros2.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>  // 引入 UTM 转换头文件


#define DEG2RAD 0.0174532925199433
#define RAD2DEG 57.29577951308232

//Normalizes the angle in radians between[-pi and pi).
static double NormalizeAngle_radian(const double& angle_radians)
{
    return angle_radians - (2.0 * M_PI) * std::floor((angle_radians + M_PI) / (2.0 * M_PI));
}

//360°版
static double NormalizeAngle_degree(const double& angle_radians)
{
    return angle_radians - (2.0 * 180.0) * std::floor((angle_radians + 180.0) / (2.0 * 180.0));
}

DataSenderROS2::DataSenderROS2(rclcpp::Node::SharedPtr node_in)
    : node_ptr(node_in)
{
    loadConfig();
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    pub_ptr = node_ptr->create_publisher<sensor_msgs::msg::NavSatFix>(topic, qos);
    data_parser_ptr = std::make_shared<DataParser>(port, baud_rate);
}

bool DataSenderROS2::run()
{
    data_parser_ptr->setCallback([&](const KSXT *msg) {
        sensor_msgs::msg::NavSatFix nav_msg;
        nav_msg.header.stamp = rclcpp::Clock().now();
        nav_msg.header.frame_id = "map";
        nav_msg.position_covariance[0] = msg->data.lat;                     // 经度
        nav_msg.position_covariance[1] = msg->data.lon;                      // 纬度
        nav_msg.altitude = msg->data.height;                   // 高度
        // 定位质量（0：定位不可用；1：单点定位；2：伪距差分/SBAS定位；4：RTK固定解；5：RTK浮点解；6：惯导定位）
        nav_msg.status.status = msg->data.pos_qual;                // 值为4时可用
        // 定向质量（0：定位不可用；1：单点定位；2：伪距差分/SBAS定位；4：RTK固定解；5：RTK浮点解；6：惯导定位）
        nav_msg.position_covariance_type = msg->data.heading_qual; // 值为4时可用
        if (msg->data.heading_qual == 4) {
            // 根据杆臂值对坐标值进行改正
            double delta_x, delta_y, x, y, lat, lon;
            int zone; bool northp;
            LL2UTM(msg->data.lat, msg->data.lon, x, y, zone, northp);
            calib(msg->data.heading, lever_arm, delta_x, delta_y);
            x+=delta_x; y+=delta_y;
            UTM2LL(x, y, zone, northp, lat, lon);
            nav_msg.latitude = lat; // 改正后的经度 °
            nav_msg.longitude = lon; // 改正后的纬度 °
            // 双天线定向方位角，单位：度，真北方向为0，顺时针为正，取值范围（0-360°）
            double heading_temp = msg->data.heading + offset_angle;
            if (heading_temp > 360)
                heading_temp -= 360.0;
            else if (heading_temp < 0)
                heading_temp += 360.0;
            nav_msg.position_covariance[2] = heading_temp;
        } else {
            nav_msg.latitude = msg->data.lat;
            nav_msg.longitude = msg->data.lon;
            // 有 GPRMC course 时输出航向（由 parser 在 heading_qual!=4 时填充）
            double heading_temp = msg->data.heading + offset_angle;
            if (heading_temp > 360)
                heading_temp -= 360.0;
            else if (heading_temp < 0)
                heading_temp += 360.0;
            nav_msg.position_covariance[2] = heading_temp;
        }
        nav_msg.position_covariance[3] = msg->data.vel_east;   // 东向速度 m/s
        nav_msg.position_covariance[4] = msg->data.vel_north;  // 北向速度 m/s
        nav_msg.position_covariance[5] = msg->data.vel_up;     // 顶向速度 m/s

        if (msg->utc.year == 0) {
            nav_msg.status.service = 0;
        } else {
            std::tm tm = {};
            tm.tm_year = msg->utc.year - 1900; // tm_year 是从 1900 开始的年数
            tm.tm_mon = msg->utc.month - 1; // tm_mon 是从 0 开始的月份
            tm.tm_mday = msg->utc.day; // tm_mday 是月份中的天数
            tm.tm_hour = msg->utc.hour; // tm_hour 是小时
            tm.tm_min = msg->utc.minute; // tm_min 是分钟
            tm.tm_sec = msg->utc.second; // tm_sec 是秒
            tm.tm_isdst = -1; // 不考虑夏令时
            time_t utc_second = std::mktime(&tm);
            nav_msg.status.service = utc_second;
        }

        pub_ptr->publish(nav_msg);
    });
    if (!data_parser_ptr->run_parse())
        return false;
    return true;
}

void DataSenderROS2::close()
{
    data_parser_ptr->close();
}

void DataSenderROS2::loadConfig()
{
    // 从配置文件或参数服务器加载配置
    // 示例：从 ROS2 参数服务器加载
    topic = node_ptr->declare_parameter<std::string>("topic", "/data_topic");
    port = node_ptr->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
    baud_rate = node_ptr->declare_parameter<int>("baud_rate", 115200);
    lever_arm = node_ptr->declare_parameter<double>("lever_arm", 0.0);
    offset_angle = node_ptr->declare_parameter<double>("offset_angle", 0.0);
    std::cout << "port: " << port << ", topic: " << topic <<", baud_rate: " << baud_rate
    << ", lever_arm: " << lever_arm << " m, offset_angle: " << offset_angle << " deg" << std::endl;
}

void DataSenderROS2::calib(const double &heading,
    const double &lever_arm, double &delta_x, double &delta_y) {

    double yaw = NormalizeAngle_degree(heading);
    delta_x = lever_arm * cos(M_PI_2 - yaw);
    delta_y = lever_arm * sin(M_PI_2 - yaw);
}

void DataSenderROS2::LL2UTM(const double &lat, const double &lon, double &x, double &y, int &zone, bool &northp) {
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);  // 使用 GeographicLib 进行 UTM 转换
}

void DataSenderROS2::UTM2LL(const double &x, const double &y, const int& zone, const bool &northp, double &lat, double &lon) {
    GeographicLib::UTMUPS::Reverse(zone, northp, x, y, lat, lon);  // 使用 GeographicLib 进行 UTM 转换
}

int main(int argc, char **argv)
{
    // 初始化 ROS2 节点
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<rclcpp::Node>("data_sender_node");

    // 创建 DataSenderROS2 实例
    DataSenderROS2 data_sender(node);

    // 启动数据发送线程
    std::thread send_thread = std::thread([&data_sender]() {
        if (!data_sender.run()) {
            RCLCPP_ERROR(data_sender.node_ptr->get_logger(), "Failed to start data sender");
        }
    });

    // 等待节点运行
    rclcpp::spin(node);

    // 关闭数据发送线程
    data_sender.close();
    if (send_thread.joinable()) {
        send_thread.join();
    }

    // 关闭 ROS2 节点
    rclcpp::shutdown();

    return 0;
}
