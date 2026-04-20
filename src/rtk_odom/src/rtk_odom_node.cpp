/**
 * @file rtk_odom_node.cpp
 * @brief 独立 RTK 订阅节点
 *
 * 订阅:
 *   - /qx/evk (sensor_msgs/NavSatFix): RTK 位置
 *   - /qx/evk/heading (std_msgs/Float64): 朝向（度）
 *
 * 过滤: 仅接受 status.status == 2 (GBAS_FIX)，即 RTK 固定解(pos_qual=4) 和 RTK 浮点解(pos_qual=5)
 *
 * 发布:
 *   - /rtk_odom (nav_msgs/Odometry): 带朝向的 RTK 里程计
 *   - /rtk_path (nav_msgs/Path): RTK 轨迹
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include <mutex>
#include <deque>
#include <vector>

// NavSatStatus 枚举 (sensor_msgs/NavSatFix)
// status.status: -1=NO_FIX, 0=FIX, 1=SBAS_FIX, 2=GBAS_FIX
// 仅 status==2 (GBAS_FIX) 对应 RTK 固定解(4) 和 RTK 浮点解(5)
constexpr int RTK_GBAS_FIX = 2;

class RtkOdomNode
{
public:
  RtkOdomNode() : geo_converter_(0, 0, 0), gnss_inited_(false), ned_align_inited_(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 参数
    pnh.param<std::string>("gnss_topic", gnss_topic_, "/qx/evk");
    pnh.param<std::string>("heading_topic", heading_topic_, "/qx/evk/heading");
    pnh.param<std::string>("frame_id", frame_id_, "camera_init");
    pnh.param<std::string>("child_frame_id", child_frame_id_, "rtk");

    // RTK-LiDAR yaw offset (deg): compensate compass-IMU heading difference
    // Set to the NEGATIVE of odom_fusion's calibrated frame_rotation
    // e.g. if fusion reports 5.76 deg, set this to -5.76
    pnh.param<double>("yaw_offset_deg", yaw_offset_deg_, 0.0);

    // GNSS 到 LiDAR 外参（默认单位）
    pnh.param<double>("extrinT_x", extrinT_(0), 0.0);
    pnh.param<double>("extrinT_y", extrinT_(1), 0.0);
    pnh.param<double>("extrinT_z", extrinT_(2), 0.0);
    extrinR_.setIdentity();

    // Preset path & lane boundary YAML files
    pnh.param<std::string>("preset_path_file", preset_path_file_, "");
    pnh.param<std::string>("lane_left_file", lane_left_file_, "");
    pnh.param<std::string>("lane_right_file", lane_right_file_, "");

    // 订阅
    sub_gnss_ = nh.subscribe(gnss_topic_, 10, &RtkOdomNode::gnssCallback, this);
    sub_heading_ = nh.subscribe(heading_topic_, 10, &RtkOdomNode::headingCallback, this);

    // 发布
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/rtk_odom", 10);
    pub_path_ = nh.advertise<nav_msgs::Path>("/rtk_path", 10);
    pub_preset_path_ = nh.advertise<nav_msgs::Path>("/preset_path", 1, true);
    pub_lane_left_ = nh.advertise<nav_msgs::Path>("/lane_boundary_left", 1, true);
    pub_lane_right_ = nh.advertise<nav_msgs::Path>("/lane_boundary_right", 1, true);

    // 加载预设路径和车道线
    preset_waypoints_ = loadWaypointsFromYAML(preset_path_file_);
    lane_left_waypoints_ = loadWaypointsFromYAML(lane_left_file_);
    lane_right_waypoints_ = loadWaypointsFromYAML(lane_right_file_);

    ROS_INFO("[rtk_odom] sub: %s, %s; pub: /rtk_odom, /rtk_path; "
             "accept only GBAS_FIX (status==2); yaw_offset=%.2f deg",
             gnss_topic_.c_str(), heading_topic_.c_str(), yaw_offset_deg_);
    if (!preset_waypoints_.empty())
      ROS_INFO("[rtk_odom] Preset path loaded: %zu waypoints", preset_waypoints_.size());
    if (!lane_left_waypoints_.empty() || !lane_right_waypoints_.empty())
      ROS_INFO("[rtk_odom] Lane boundaries loaded: left=%zu, right=%zu",
               lane_left_waypoints_.size(), lane_right_waypoints_.size());
  }

private:
  void headingCallback(const std_msgs::Float64ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mtx_heading_);
    rtk_heading_deg_ = msg->data;
    rtk_heading_received_ = true;
  }

  void gnssCallback(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    // 过滤：仅接受 status.status == 2 (GBAS_FIX)，即 RTK 固定解和浮点解
    if (msg->status.status != RTK_GBAS_FIX)
    {
      ROS_DEBUG_THROTTLE(5.0, "RTK filter: status=%d (need 2/GBAS_FIX), skip", msg->status.status);
      return;
    }

    double timestamp = msg->header.stamp.toSec();

    // 协方差 (ENU 顺序: [0]=E, [4]=N, [8]=U)
    double cov_e = msg->position_covariance[0];
    double cov_n = msg->position_covariance[4];
    double cov_u = msg->position_covariance[8];

    if (!gnss_inited_)
    {
      geo_converter_.Reset(msg->latitude, msg->longitude, msg->altitude);
      gnss_inited_ = true;
      ROS_INFO("[rtk_odom] Origin init: lat=%.6f, lon=%.6f, alt=%.2f",
               msg->latitude, msg->longitude, msg->altitude);
      return;
    }

    // NED 对齐：首次有效位置时用当前航向建立 camera_init 系
    if (!ned_align_inited_)
    {
      std::lock_guard<std::mutex> lock(mtx_heading_);
      init_heading_deg_ = rtk_heading_received_ ? rtk_heading_deg_ : 0.0;
      // /qx/evk/heading 为天线朝向(尾部), 加180°为前进方向
      // yaw_offset_deg_ compensates compass-IMU heading difference
      double h = (init_heading_deg_ + 180.0 + yaw_offset_deg_) * M_PI / 180.0;
      // camera_init: x=前, y=左, z=上; NED: N=前, E=右, D=下
      R_cam_from_ned_ << std::cos(h),  std::sin(h), 0.0,
                         std::sin(h), -std::cos(h), 0.0,
                         0.0,          0.0,        -1.0;
      ned_align_inited_ = true;
      ROS_INFO("[rtk_odom] NED aligned to camera_init, init heading: %.2f deg", init_heading_deg_);

      // NED 对齐完成，可以转换并发布预设路径
      convertAndPublishPresetPath();
    }

    // WGS84 -> 局部 ENU (GeographicLib 的 Forward 输出 E,N,U)
    double local_E, local_N, local_U;
    geo_converter_.Forward(msg->latitude, msg->longitude, msg->altitude, local_E, local_N, local_U);

    // NED 位置: N=local_N, E=local_E, D=-local_U
    Eigen::Vector3d ned_pos(local_N, local_E, -local_U);
    Eigen::Vector3d cam_pos = R_cam_from_ned_ * ned_pos;

    // GNSS -> LiDAR 外参变换
    Eigen::Vector3d pos_final = extrinR_ * cam_pos + extrinT_;

    // 相对航向 (camera_init 系下 yaw), include yaw_offset to match LIO frame
    double relative_yaw_rad;
    {
      std::lock_guard<std::mutex> lock(mtx_heading_);
      relative_yaw_rad = (init_heading_deg_ - rtk_heading_deg_ + yaw_offset_deg_) * M_PI / 180.0;
    }

    // 发布 /rtk_odom
    nav_msgs::Odometry rtk_odom;
    rtk_odom.header.stamp = msg->header.stamp;
    rtk_odom.header.frame_id = frame_id_;
    rtk_odom.child_frame_id = child_frame_id_;
    rtk_odom.pose.pose.position.x = pos_final(0);
    rtk_odom.pose.pose.position.y = pos_final(1);
    rtk_odom.pose.pose.position.z = pos_final(2);
    double half_yaw = relative_yaw_rad / 2.0;
    rtk_odom.pose.pose.orientation.x = 0.0;
    rtk_odom.pose.pose.orientation.y = 0.0;
    rtk_odom.pose.pose.orientation.z = std::sin(half_yaw);
    rtk_odom.pose.pose.orientation.w = std::cos(half_yaw);
    rtk_odom.pose.covariance[0] = cov_e;
    rtk_odom.pose.covariance[7] = cov_n;
    rtk_odom.pose.covariance[14] = cov_u;
    rtk_odom.pose.covariance[35] = 0.1;  // heading 协方差
    pub_odom_.publish(rtk_odom);

    // 发布轨迹
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = rtk_odom.header;
    pose_stamped.pose = rtk_odom.pose.pose;
    rtk_path_.poses.push_back(pose_stamped);
    rtk_path_.header = pose_stamped.header;
    pub_path_.publish(rtk_path_);
  }

  struct GpsWaypoint
  {
    double lat, lon, alt;
  };

  std::vector<GpsWaypoint> loadWaypointsFromYAML(const std::string& filepath)
  {
    std::vector<GpsWaypoint> waypoints;
    if (filepath.empty()) return waypoints;
    try
    {
      YAML::Node doc = YAML::LoadFile(filepath);
      if (!doc["path"]) { ROS_WARN("[rtk_odom] No 'path' key in %s", filepath.c_str()); return waypoints; }
      for (const auto& pt : doc["path"])
      {
        GpsWaypoint wp;
        wp.lat = pt["latitude"].as<double>();
        wp.lon = pt["longitude"].as<double>();
        wp.alt = pt["altitude"].as<double>();
        waypoints.push_back(wp);
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[rtk_odom] Failed to load %s: %s", filepath.c_str(), e.what());
    }
    return waypoints;
  }

  nav_msgs::Path convertWaypointsToPath(const std::vector<GpsWaypoint>& waypoints)
  {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;
    for (const auto& wp : waypoints)
    {
      double local_E, local_N, local_U;
      geo_converter_.Forward(wp.lat, wp.lon, wp.alt, local_E, local_N, local_U);
      Eigen::Vector3d ned_pos(local_N, local_E, -local_U);
      Eigen::Vector3d cam_pos = R_cam_from_ned_ * ned_pos;
      Eigen::Vector3d pos_final = extrinR_ * cam_pos + extrinT_;
      geometry_msgs::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pos_final(0);
      ps.pose.position.y = pos_final(1);
      ps.pose.position.z = pos_final(2);
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    return path;
  }

  void convertAndPublishPresetPath()
  {
    if (!preset_waypoints_.empty())
    {
      nav_msgs::Path msg = convertWaypointsToPath(preset_waypoints_);
      pub_preset_path_.publish(msg);
      ROS_INFO("\033[1;32m[rtk_odom] Preset path published: %zu points on /preset_path\033[0m", msg.poses.size());
    }
    if (!lane_left_waypoints_.empty())
    {
      nav_msgs::Path msg = convertWaypointsToPath(lane_left_waypoints_);
      pub_lane_left_.publish(msg);
      ROS_INFO("\033[1;32m[rtk_odom] Lane left published: %zu points\033[0m", msg.poses.size());
    }
    if (!lane_right_waypoints_.empty())
    {
      nav_msgs::Path msg = convertWaypointsToPath(lane_right_waypoints_);
      pub_lane_right_.publish(msg);
      ROS_INFO("\033[1;32m[rtk_odom] Lane right published: %zu points\033[0m", msg.poses.size());
    }
  }

  ros::Subscriber sub_gnss_;
  ros::Subscriber sub_heading_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_path_;
  ros::Publisher pub_preset_path_;
  ros::Publisher pub_lane_left_;
  ros::Publisher pub_lane_right_;

  std::string gnss_topic_;
  std::string heading_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  GeographicLib::LocalCartesian geo_converter_;
  bool gnss_inited_;
  bool ned_align_inited_;
  Eigen::Matrix3d R_cam_from_ned_;
  Eigen::Vector3d extrinT_;
  Eigen::Matrix3d extrinR_;

  std::mutex mtx_heading_;
  double rtk_heading_deg_ = 0.0;
  bool rtk_heading_received_ = false;
  double init_heading_deg_ = 0.0;
  double yaw_offset_deg_ = 0.0;

  // Preset path & lane boundaries
  std::string preset_path_file_;
  std::string lane_left_file_;
  std::string lane_right_file_;
  std::vector<GpsWaypoint> preset_waypoints_;
  std::vector<GpsWaypoint> lane_left_waypoints_;
  std::vector<GpsWaypoint> lane_right_waypoints_;

  nav_msgs::Path rtk_path_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtk_odom_node");
  RtkOdomNode node;
  ros::spin();
  return 0;
}
