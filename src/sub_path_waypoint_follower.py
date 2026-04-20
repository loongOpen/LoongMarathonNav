#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1版本：订阅/target_path，动态前视点跟随
逻辑：根据机器人当前位置找到最近路径点，再找到距离>lookahead的目标点
发布格式与 waypoint_example 一致：geometry_msgs/PointStamped 到 /way_point
"""

import argparse
import math
import sys

import numpy as np

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class OdinPathFollowerRos1(object):
    def __init__(self, args):
        rospy.init_node("path_follower", anonymous=False)

        # 参数（优先使用命令行，其次使用ROS参数）
        self.path_topic = self._get_param(args, "path_topic", "preset_path")
        self.global_frame = self._get_param(args, "global_frame", "map")
        self.base_frame = self._get_param(args, "base_frame", "sensor")
        self.lookahead_distance = self._get_param(args, "lookahead_distance", 4.0)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 路径数据
        self.current_path = None
        self.path_received = False
        self.waypoints = []

        # 订阅路径
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        
        # 发布目标点可视化 marker
        self.marker_pub = rospy.Publisher("goal_marker", Marker, queue_size=1)
        
        # 发布目标点到 /way_point（local_planner 订阅这个话题）
        self.waypoint_pub = rospy.Publisher("/way_point", PointStamped, queue_size=5)

        rospy.loginfo("Path follower initialized, publishing to /way_point")

    @staticmethod
    def _get_param(args, name, default):
        value = getattr(args, name, None)
        if value is not None:
            return value
        return rospy.get_param("~" + name, default)

    def path_callback(self, msg):
        if self.path_received:
            return
        self.current_path = msg
        self.path_received = True
        rospy.loginfo("Received path with %d poses", len(msg.poses))

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.1),
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            return x, y, z
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as ex:
            rospy.logwarn_throttle(5.0, "TF lookup failed: %s", str(ex))
            return None, None, None

    def build_waypoints(self):
        """从路径构建航点列表和沿路径的累积弧长"""
        poses = self.current_path.poses
        n = len(poses)
        self.wp_xy = np.empty((n, 2), dtype=np.float64)
        self.wp_z = np.empty(n, dtype=np.float64)
        self.waypoints = []
        for i, p in enumerate(poses):
            self.wp_xy[i, 0] = p.pose.position.x
            self.wp_xy[i, 1] = p.pose.position.y
            self.wp_z[i] = p.pose.position.z
            point = PointStamped()
            point.header.frame_id = self.global_frame
            point.point.x = p.pose.position.x
            point.point.y = p.pose.position.y
            point.point.z = p.pose.position.z
            self.waypoints.append(point)

        # 预计算沿路径的累积弧长，用于按路径前进距离查找目标点
        diffs = np.diff(self.wp_xy, axis=0)
        seg_lengths = np.sqrt(diffs[:, 0] ** 2 + diffs[:, 1] ** 2)
        self.cumulative_dist = np.zeros(n, dtype=np.float64)
        self.cumulative_dist[1:] = np.cumsum(seg_lengths)

    def send_waypoint(self, waypoint):
        """发布航点到 /way_point（与 waypoint_example 格式一致）"""
        # 创建 PointStamped 消息
        waypoint_msg = PointStamped()
        waypoint_msg.header.frame_id = self.global_frame
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.point.x = waypoint.point.x
        waypoint_msg.point.y = waypoint.point.y
        waypoint_msg.point.z = waypoint.point.z
        
        # 发布航点
        self.waypoint_pub.publish(waypoint_msg)

        # 发布可视化 marker（球形）
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = waypoint.point.x
        marker.pose.position.y = waypoint.point.y
        marker.pose.position.z = waypoint.point.z
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.lifetime = rospy.Duration(0)
        
        self.marker_pub.publish(marker)

    def run(self):
        # 等待路径
        rospy.loginfo("Waiting for %s ...", self.path_topic)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.path_received:
            rate.sleep()

        if self.current_path is None or len(self.current_path.poses) == 0:
            rospy.logerr("No path received, exiting")
            return

        self.build_waypoints()
        num_points = len(self.waypoints)

        path_progress = 0
        BACK_WINDOW = 5
        FWD_WINDOW = 80

        current_goal_index = -1
        log_counter = 0
        last_publish_time = rospy.Time.now()
        publish_interval = rospy.Duration(0.2)

        while not rospy.is_shutdown():
            x, y, z = self.get_robot_position()
            if x is None:
                rospy.sleep(0.05)
                continue

            dxy = self.wp_xy - np.array([x, y])
            dist_sq = dxy[:, 0] * dxy[:, 0] + dxy[:, 1] * dxy[:, 1]

            # 1) 在当前进度附近的窗口内搜索最近点，防止跳到其他路段
            search_start = max(0, path_progress - BACK_WINDOW)
            search_end = min(num_points, path_progress + FWD_WINDOW)
            local_nearest = int(np.argmin(dist_sq[search_start:search_end]))
            nearest_index = search_start + local_nearest

            # 只允许前进（或微小后退），不允许大幅回退
            if nearest_index >= path_progress - BACK_WINDOW:
                path_progress = max(path_progress, nearest_index)

            # 如果窗口内最近点距离太远（偏离路径），扩大搜索到全局
            if dist_sq[nearest_index] > 100.0:  # >10m，可能初始化或严重偏离
                nearest_index = int(np.argmin(dist_sq))
                path_progress = nearest_index

            # 2) 沿路径弧长前进 lookahead_distance 找目标点
            target_arc = self.cumulative_dist[path_progress] + self.lookahead_distance
            ahead = np.where(self.cumulative_dist[path_progress:] >= target_arc)[0]
            if ahead.size > 0:
                target_index = path_progress + int(ahead[0])
            else:
                target_index = num_points - 1

            # 2.1) 到达末端时，从头循环寻找
            if target_index == num_points - 1:
                remaining = target_arc - self.cumulative_dist[-1]
                if remaining > 0:
                    wrap = np.where(self.cumulative_dist >= remaining)[0]
                    if wrap.size > 0:
                        target_index = int(wrap[0])
                        path_progress = 0

            # 3) 按固定频率发布航点
            now = rospy.Time.now()
            if now - last_publish_time > publish_interval:
                target_waypoint = self.waypoints[target_index]

                if target_index != current_goal_index:
                    current_goal_index = target_index
                self.send_waypoint(target_waypoint)
                last_publish_time = now

            # 4) 状态日志
            log_counter += 1
            if log_counter % 100 == 0:
                wp = self.waypoints[target_index].point
                dist = math.hypot(x - wp.x, y - wp.y)

            rospy.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(
        description="ROS1 路径跟随节点，发布 /way_point 给 local_planner"
    )
    parser.add_argument("--path_topic", default=None, help="订阅的路径话题")
    parser.add_argument("--global_frame", default=None, help="全局坐标系")
    parser.add_argument("--base_frame", default=None, help="机器人坐标系")
    parser.add_argument("--lookahead_distance", type=float, default=None, help="前视距离")
    args, _ = parser.parse_known_args()

    try:
        node = OdinPathFollowerRos1(args)
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
