# 千寻位置 EVK 定位定向设备 ROS 驱动

读取千寻位置 EVK 设备的 KSXT 串口数据，解析后发布为标准 ROS 消息。
支持 ROS1（Noetic/Melodic）和 ROS2（Humble/Iron/Rolling/Jazzy）。

## 依赖

```bash
sudo apt install libgeographiclib-dev  # GeographicLib
# boost 通常随 ROS 一起安装，无需单独安装
```

## 支持的 ROS 版本

| ROS 版本 | 发行版 | 编译方式 | 最低 Ubuntu |
|----------|--------|----------|-------------|
| ROS1 | Noetic | catkin_make | 20.04 |
| ROS1 | Melodic | catkin_make | 18.04 |
| ROS2 | Humble | colcon build | 22.04 |
| ROS2 | Iron | colcon build | 22.04 |
| ROS2 | Rolling | colcon build | 22.04 |
| ROS2 | Jazzy | colcon build | 24.04 |

## 编译

**ROS1：**
```bash
source /opt/ros/noetic/setup.bash
cd ~/your_ws
catkin_make --pkg qx_evk_driver
```

**ROS2：**
```bash
source /opt/ros/humble/setup.bash
cd ~/your_ws
colcon build --packages-select qx_evk_driver
```

CMakeLists.txt 会根据 `$ROS_VERSION` 环境变量自动选择编译方式。

## 配置

编辑 `config/qx_evk_cfg.yaml`：

```yaml
serial_port: /dev/ttyUSB0  # 串口设备
topic: /qx/evk             # NavSatFix 话题名（速度/航向话题在此基础上加后缀）
baud_rate: 115200          # 波特率
lever_arm: 0.00            # 主天线到标定点的距离（m），仅双天线定向有效时生效
offset_angle: 0.00         # 天线安装航向偏移（°），叠加到输出航向角上
```

## 运行

**ROS1：**
```bash
source devel/setup.bash
roslaunch qx_evk_driver qx_evk_driver.launch
```

**ROS2：**
```bash
source install/setup.bash
ros2 launch qx_evk_driver qx_evk_driver.py
```

### 串口权限问题

若提示 `Permission denied`，选择以下任一方案：

```bash
# 永久方案（推荐，重启后生效）
sudo usermod -aG dialout $USER

# 临时方案
sudo chmod 666 /dev/ttyUSB0
```

## 发布的话题

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/qx/evk` | `sensor_msgs/NavSatFix` | 定位信息（标准格式） |
| `/qx/evk/velocity` | `geometry_msgs/TwistStamped` | ENU 三轴速度（m/s） |
| `/qx/evk/heading` | `std_msgs/Float64` | 航向角（°） |

> 话题名可在配置文件中通过 `topic` 参数修改，速度和航向话题自动以 `/velocity`、`/heading` 为后缀。

## 消息字段说明

### sensor_msgs/NavSatFix（`/qx/evk`）

| 字段 | 说明 |
|------|------|
| `header.stamp` | 接收到数据时的系统时间戳 |
| `header.frame_id` | `"wgs84"` |
| `latitude` | 纬度（°，WGS84），双天线定向有效时已施加杆臂改正 |
| `longitude` | 经度（°，WGS84），双天线定向有效时已施加杆臂改正 |
| `altitude` | 椭球高（m） |
| `status.status` | 定位状态（见下表） |
| `status.service` | 卫星系统位掩码：`5` = GPS(1) \| BDS(4) |
| `position_covariance` | 位置协方差对角阵（m²，ENU 顺序），按定位质量近似给出 |
| `position_covariance_type` | `0`=未知，`1`=近似（当前） |

**`status.status` 映射：**

| KSXT pos_qual | NavSatStatus 值 | 含义 |
|---------------|----------------|------|
| 0 | -1（NO_FIX） | 定位不可用 |
| 1 | 0（FIX） | 单点定位 |
| 2 | 1（SBAS_FIX） | 伪距差分 / SBAS |
| 4 | 2（GBAS_FIX） | RTK 固定解 |
| 5 | 2（GBAS_FIX） | RTK 浮点解 |
| 6 | 0（FIX） | 惯导定位 |

**`position_covariance` 近似值：**

| 定位质量 | east/north 方差（m²） | up 方差（m²） | 对应精度 |
|---------|----------------------|--------------|---------|
| RTK 固定解（4） | 0.0004 | 0.0016 | ±0.02 m / ±0.04 m |
| RTK 浮点解（5） | 0.09 | 0.25 | ±0.3 m / ±0.5 m |
| 惯导（6） | 0.25 | 1.0 | ±0.5 m / ±1.0 m |
| SBAS 差分（2） | 1.0 | 4.0 | ±1.0 m / ±2.0 m |
| 单点（1） | 9.0 | 25.0 | ±3.0 m / ±5.0 m |

### geometry_msgs/TwistStamped（`/qx/evk/velocity`）

ENU 坐标系下的地面速度，`angular` 全为零。

| 字段 | 说明 |
|------|------|
| `twist.linear.x` | 东向速度（m/s） |
| `twist.linear.y` | 北向速度（m/s） |
| `twist.linear.z` | 天向速度（m/s） |

### std_msgs/Float64（`/qx/evk/heading`）

航向角（°），真北为 0°，顺时针为正，范围 [0, 360)，已叠加 `offset_angle` 安装偏移。

双天线定向质量有效（`heading_qual=4`）时为双天线定向航向，否则为 GPRMC 航迹角（设备静止时可能抖动，仅供参考）。
