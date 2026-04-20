#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string stateEstimationTopic = "/fused_odom";
string registeredScanTopic = "/cloud_registered_body";
bool flipStateEstimation = false;
bool flipRegisteredScan = false;
bool sendTF = true;
bool reverseTF = false;
double downsampleLeafSize = 0.5;
double minScanRange = 1.5;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

ros::Publisher *pubOdometryPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *pubLaserCloudPointer = NULL;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;

void syncedCallback(const nav_msgs::Odometry::ConstPtr& odom,
                    const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  // --- 1. 用同步的里程计构建 body→world 变换 ---
  tf::Transform odomPose;
  odomPose.setRotation(tf::Quaternion(
      odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z, odom->pose.pose.orientation.w));
  odomPose.setOrigin(tf::Vector3(
      odom->pose.pose.position.x, odom->pose.pose.position.y,
      odom->pose.pose.position.z));

  // --- 2. 发布 /state_estimation（与点云变换使用完全相同的位姿） ---
  nav_msgs::Odometry odomData = *odom;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;

  if (flipStateEstimation) {
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
    pitch = -pitch;
    yaw = -yaw;
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = odom->pose.pose.position.z;
    odomData.pose.pose.position.y = odom->pose.pose.position.x;
    odomData.pose.pose.position.z = odom->pose.pose.position.y;
  }

  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";
  pubOdometryPointer->publish(odomData);

  if (sendTF) {
    tf::StampedTransform odomTrans;
    odomTrans.stamp_ = odom->header.stamp;
    odomTrans.frame_id_ = "map";
    odomTrans.child_frame_id_ = "sensor";
    odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));
    if (!reverseTF) {
      tfBroadcasterPointer->sendTransform(odomTrans);
    } else {
      tfBroadcasterPointer->sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "sensor", "map"));
    }
  }

  // --- 3. 解析点云，过滤近距离点，降采样 ---
  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  if (flipRegisteredScan) {
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      float temp = laserCloud->points[i].x;
      laserCloud->points[i].x = laserCloud->points[i].z;
      laserCloud->points[i].z = laserCloud->points[i].y;
      laserCloud->points[i].y = temp;
    }
  }

  // 过滤 body 帧中距离传感器太近的点（机器人本体反射、近距离噪声）
  double minRangeSq = minScanRange * minScanRange;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFiltered(new pcl::PointCloud<pcl::PointXYZI>());
  laserCloudFiltered->reserve(laserCloud->points.size());
  for (size_t i = 0; i < laserCloud->points.size(); i++) {
    float x = laserCloud->points[i].x;
    float y = laserCloud->points[i].y;
    float z = laserCloud->points[i].z;
    if (x * x + y * y + z * z > minRangeSq) {
      laserCloudFiltered->push_back(laserCloud->points[i]);
    }
  }

  // body 帧降采样（匹配 FAST-LIO 的 feats_down_body 密度）
  laserCloudDwz->clear();
  downSizeFilter.setInputCloud(laserCloudFiltered);
  downSizeFilter.filter(*laserCloudDwz);

  // --- 4. body 帧 → 世界坐标系 ---
  int cloudSize = laserCloudDwz->points.size();
  for (int i = 0; i < cloudSize; i++) {
    tf::Vector3 pt(laserCloudDwz->points[i].x,
                   laserCloudDwz->points[i].y,
                   laserCloudDwz->points[i].z);
    tf::Vector3 ptWorld = odomPose * pt;
    laserCloudDwz->points[i].x = ptWorld.x();
    laserCloudDwz->points[i].y = ptWorld.y();
    laserCloudDwz->points[i].z = ptWorld.z();
  }

  sensor_msgs::PointCloud2 laserCloud2;
  pcl::toROSMsg(*laserCloudDwz, laserCloud2);
  laserCloud2.header.stamp = odom->header.stamp;
  laserCloud2.header.frame_id = "map";
  pubLaserCloudPointer->publish(laserCloud2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);
  nhPrivate.getParam("downsampleLeafSize", downsampleLeafSize);
  nhPrivate.getParam("minScanRange", minScanRange);

  downSizeFilter.setLeafSize(downsampleLeafSize, downsampleLeafSize, downsampleLeafSize);

  message_filters::Subscriber<nav_msgs::Odometry> subOdometry(nh, stateEstimationTopic, 5);
  message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud(nh, registeredScanTopic, 5);

  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(50), subOdometry, subLaserCloud);
  sync.registerCallback(boost::bind(syncedCallback, _1, _2));

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);
  pubLaserCloudPointer = &pubLaserCloud;

  ros::spin();

  return 0;
}
