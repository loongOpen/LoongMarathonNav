/**
 * @file odom_fusion_node.cpp
 * @brief RTK + LIO fusion with online frame rotation calibration
 *
 * Key insight: LIO's camera_init (IMU-aligned) and RTK's camera_init
 * (compass-aligned) are rotated relative to each other. The rotation angle
 * cannot be measured from heading alone (both start at 0). Instead we compare
 * the displacement DIRECTIONS when both sources are active.
 *
 * Frame rotation estimation:
 *   Sliding window of (atan2(dRTK) - atan2(dLIO)) samples when the robot moves.
 *   This single value captures both the fixed extrinsic rotation AND any
 *   accumulated LIO heading drift, updated continuously.
 *
 * Position: fused = anchor_rtk + Rz(frame_rotation) * (lio - anchor_lio)
 * Heading:  fused_yaw = lio_yaw + frame_rotation
 * Roll/Pitch: from LIO directly
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <deque>
#include <mutex>
#include <cmath>
#include <vector>

namespace {

double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double quatToYaw(const geometry_msgs::Quaternion& q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

void quatToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

geometry_msgs::Quaternion rpyToQuat(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion out;
  tf2::convert(q, out);
  return out;
}

double cosineDecay(double elapsed, double duration)
{
  if (elapsed >= duration) return 0.0;
  return 0.5 * (1.0 + std::cos(M_PI * elapsed / duration));
}

double circularMean(const std::vector<double>& angles)
{
  double ss = 0.0, sc = 0.0;
  for (double a : angles) { ss += std::sin(a); sc += std::cos(a); }
  return std::atan2(ss, sc);
}

}  // namespace

struct LioPose
{
  double t;
  Eigen::Vector3d pos;
  double yaw;
};

struct DispSample
{
  double t;
  double angle_diff;
};

class OdomFusionNode
{
public:
  OdomFusionNode()
    : anchor_initialized_(false)
    , rtk_available_(false)
    , in_recovery_(false)
    , last_rtk_time_(0.0)
    , frame_rotation_(0.0)
    , frame_rotation_valid_(false)
    , prev_disp_valid_(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("rtk_timeout", rtk_timeout_, 0.5);
    pnh.param<double>("recovery_duration", recovery_duration_, 1.5);
    pnh.param<double>("lio_buffer_duration", lio_buffer_duration_, 2.0);
    pnh.param<double>("min_calib_displacement", min_calib_disp_, 0.5);
    pnh.param<double>("rotation_window_duration", rotation_window_dur_, 10.0);
    pnh.param<int>("rotation_min_samples", rotation_min_samples_, 5);
    pnh.param<std::string>("frame_id", frame_id_, "camera_init");
    pnh.param<std::string>("child_frame_id", child_frame_id_, "fused");
    pnh.param<std::string>("rtk_topic", rtk_topic_, "/rtk_odom");
    pnh.param<std::string>("lio_topic", lio_topic_, "/Odometry");

    sub_rtk_ = nh.subscribe(rtk_topic_, 10, &OdomFusionNode::rtkCallback, this);
    sub_lio_ = nh.subscribe(lio_topic_, 50, &OdomFusionNode::lioCallback, this);

    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/fused_odom", 10);
    pub_path_ = nh.advertise<nav_msgs::Path>("/fused_path", 10);

    ROS_INFO("[odom_fusion] frame rotation from displacement dirs | "
             "min_disp=%.2fm, window=%.0fs, min_samples=%d | "
             "rtk_timeout=%.2fs, recovery=%.2fs",
             min_calib_disp_, rotation_window_dur_, rotation_min_samples_,
             rtk_timeout_, recovery_duration_);
  }

private:
  bool interpolateLioAt(double t_query, Eigen::Vector3d& pos_out, double& yaw_out)
  {
    if (lio_buffer_.size() < 2) return false;
    if (t_query < lio_buffer_.front().t || t_query > lio_buffer_.back().t) return false;

    for (size_t i = 0; i + 1 < lio_buffer_.size(); ++i)
    {
      if (lio_buffer_[i].t <= t_query && t_query <= lio_buffer_[i + 1].t)
      {
        double dt = lio_buffer_[i + 1].t - lio_buffer_[i].t;
        if (dt < 1e-6)
        {
          pos_out = lio_buffer_[i].pos;
          yaw_out = lio_buffer_[i].yaw;
          return true;
        }
        double r = (t_query - lio_buffer_[i].t) / dt;
        pos_out = (1.0 - r) * lio_buffer_[i].pos + r * lio_buffer_[i + 1].pos;
        double dy = normalizeAngle(lio_buffer_[i + 1].yaw - lio_buffer_[i].yaw);
        yaw_out = normalizeAngle(lio_buffer_[i].yaw + r * dy);
        return true;
      }
    }
    return false;
  }

  Eigen::Vector3d rotateDelta(const Eigen::Vector3d& delta) const
  {
    double c = std::cos(frame_rotation_), s = std::sin(frame_rotation_);
    return Eigen::Vector3d(
        c * delta(0) - s * delta(1),
        s * delta(0) + c * delta(1),
        delta(2));
  }

  Eigen::Vector3d getCurrentPosOutput(double t, const Eigen::Vector3d& lio_pos)
  {
    Eigen::Vector3d out = anchor_rtk_pos_ + rotateDelta(lio_pos - anchor_lio_pos_);
    if (in_recovery_)
      out += cosineDecay(t - recovery_start_time_, recovery_duration_) * recovery_jump_pos_;
    return out;
  }

  // Compare displacement directions between RTK and LIO to estimate frame rotation
  void updateFrameRotation(double t,
                           const Eigen::Vector3d& pos_rtk,
                           const Eigen::Vector3d& pos_lio)
  {
    if (!prev_disp_valid_)
    {
      prev_disp_rtk_ = pos_rtk;
      prev_disp_lio_ = pos_lio;
      prev_disp_valid_ = true;
      return;
    }

    Eigen::Vector2d d_rtk = (pos_rtk - prev_disp_rtk_).head<2>();
    Eigen::Vector2d d_lio = (pos_lio - prev_disp_lio_).head<2>();

    if (d_rtk.norm() < min_calib_disp_ || d_lio.norm() < min_calib_disp_)
      return;

    double rtk_dir = std::atan2(d_rtk(1), d_rtk(0));
    double lio_dir = std::atan2(d_lio(1), d_lio(0));
    double diff = normalizeAngle(rtk_dir - lio_dir);

    disp_window_.push_back({t, diff});
    prev_disp_rtk_ = pos_rtk;
    prev_disp_lio_ = pos_lio;

    // Trim old samples
    while (!disp_window_.empty() && (t - disp_window_.front().t) > rotation_window_dur_)
      disp_window_.pop_front();

    if (static_cast<int>(disp_window_.size()) >= rotation_min_samples_)
    {
      std::vector<double> angles;
      angles.reserve(disp_window_.size());
      for (const auto& s : disp_window_)
        angles.push_back(s.angle_diff);

      double new_rotation = circularMean(angles);

      if (!frame_rotation_valid_)
      {
        frame_rotation_ = new_rotation;
        frame_rotation_valid_ = true;
        ROS_INFO("\033[1;32m[odom_fusion] Frame rotation calibrated: %.2f deg "
                 "(%zu samples)\033[0m",
                 frame_rotation_ * 180.0 / M_PI, angles.size());
      }
      else
      {
        frame_rotation_ = new_rotation;
      }
    }
  }

  void rtkCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    double t = msg->header.stamp.toSec();

    Eigen::Vector3d pos_rtk(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);

    Eigen::Vector3d pos_lio;
    double yaw_lio;
    if (!interpolateLioAt(t, pos_lio, yaw_lio))
    {
      if (lio_buffer_.empty()) return;
      pos_lio = lio_buffer_.back().pos;
    }

    // ==================== Position anchor ====================
    if (!anchor_initialized_)
    {
      anchor_rtk_pos_ = pos_rtk;
      anchor_lio_pos_ = pos_lio;
      anchor_initialized_ = true;
      last_rtk_time_ = t;
      rtk_available_ = true;
      ROS_INFO("[odom_fusion] Anchor init: rtk=(%.2f,%.2f,%.2f)",
               pos_rtk(0), pos_rtk(1), pos_rtk(2));
    }
    else
    {
      bool was_lost = !rtk_available_ || (t - last_rtk_time_ > rtk_timeout_);
      last_rtk_time_ = t;
      rtk_available_ = true;

      if (was_lost)
      {
        Eigen::Vector3d current_out = getCurrentPosOutput(t, pos_lio);
        recovery_jump_pos_ = current_out - pos_rtk;
        recovery_start_time_ = t;
        in_recovery_ = true;
        ROS_INFO("[odom_fusion] RTK recovered, pos jump: (%.3f,%.3f,%.3f) m",
                 recovery_jump_pos_(0), recovery_jump_pos_(1), recovery_jump_pos_(2));
      }

      anchor_rtk_pos_ = pos_rtk;
      anchor_lio_pos_ = pos_lio;
    }

    // ==================== Frame rotation calibration ====================
    updateFrameRotation(t, pos_rtk, pos_lio);
  }

  void lioCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    double t = msg->header.stamp.toSec();

    LioPose lp;
    lp.t = t;
    lp.pos = Eigen::Vector3d(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    lp.yaw = quatToYaw(msg->pose.pose.orientation);
    lio_buffer_.push_back(lp);
    while (lio_buffer_.size() > 1 &&
           (lio_buffer_.back().t - lio_buffer_.front().t) > lio_buffer_duration_)
    {
      lio_buffer_.pop_front();
    }

    if (rtk_available_ && (t - last_rtk_time_ > rtk_timeout_))
    {
      rtk_available_ = false;
      ROS_WARN_THROTTLE(1.0, "[odom_fusion] RTK timeout %.2fs, dead reckoning", rtk_timeout_);
    }

    if (!anchor_initialized_)
    {
      pub_odom_.publish(*msg);
      return;
    }

    // === Position: XY from RTK+LIO fusion, Z from LIO directly ===
    Eigen::Vector3d fused_pos = anchor_rtk_pos_ + rotateDelta(lp.pos - anchor_lio_pos_);
    fused_pos(2) = lp.pos(2);  // LIO Z tracks altitude far better than GPS

    if (in_recovery_)
    {
      double elapsed = t - recovery_start_time_;
      if (elapsed >= recovery_duration_)
        in_recovery_ = false;
      else
      {
        Eigen::Vector3d jump = cosineDecay(elapsed, recovery_duration_) * recovery_jump_pos_;
        jump(2) = 0.0;  // don't apply Z recovery jump since Z comes from LIO
        fused_pos += jump;
      }
    }

    // === Heading: lio_yaw + frame_rotation (consistent with position direction) ===
    double lio_roll, lio_pitch, lio_yaw;
    quatToRPY(msg->pose.pose.orientation, lio_roll, lio_pitch, lio_yaw);
    double fused_yaw = normalizeAngle(lio_yaw + frame_rotation_);

    nav_msgs::Odometry fused;
    fused.header = msg->header;
    fused.header.frame_id = frame_id_;
    fused.child_frame_id = child_frame_id_;
    fused.pose.pose.position.x = fused_pos(0);
    fused.pose.pose.position.y = fused_pos(1);
    fused.pose.pose.position.z = fused_pos(2);
    fused.pose.pose.orientation = rpyToQuat(lio_roll, lio_pitch, fused_yaw);
    fused.pose.covariance = msg->pose.covariance;
    fused.twist = msg->twist;
    pub_odom_.publish(fused);

    geometry_msgs::PoseStamped ps;
    ps.header = fused.header;
    ps.pose = fused.pose.pose;
    path_.poses.push_back(ps);
    path_.header = ps.header;
    pub_path_.publish(path_);
  }

  ros::Subscriber sub_rtk_;
  ros::Subscriber sub_lio_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_path_;

  std::mutex mtx_;
  std::deque<LioPose> lio_buffer_;

  // Position anchor
  Eigen::Vector3d anchor_rtk_pos_;
  Eigen::Vector3d anchor_lio_pos_;
  bool anchor_initialized_;

  // RTK state
  bool rtk_available_;
  double last_rtk_time_;

  // Position recovery smoothing
  bool in_recovery_;
  double recovery_start_time_;
  Eigen::Vector3d recovery_jump_pos_;

  // Frame rotation: estimated from displacement direction comparison
  double frame_rotation_;
  bool frame_rotation_valid_;
  std::deque<DispSample> disp_window_;
  Eigen::Vector3d prev_disp_rtk_, prev_disp_lio_;
  bool prev_disp_valid_;

  // Parameters
  double rtk_timeout_;
  double recovery_duration_;
  double lio_buffer_duration_;
  double min_calib_disp_;
  double rotation_window_dur_;
  int rotation_min_samples_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string rtk_topic_;
  std::string lio_topic_;

  nav_msgs::Path path_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_fusion_node");
  OdomFusionNode node;
  ros::spin();
  return 0;
}
