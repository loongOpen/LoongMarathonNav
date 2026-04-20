#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 读取配置文件获取串口信息
get_serial_port() {
    local config_file="${SCRIPT_DIR}/../config/qx_evk_cfg.yaml"
    if [ -f "$config_file" ]; then
        local port=$(grep -oP "port:\s*\K[^ ]+" "$config_file" | head -1)
        if [ -n "$port" ]; then
            echo "$port"
            return 0
        fi
    fi
    # 默认串口
    echo "/dev/ttyUSB0"
}

# 检查和修复串口权限
check_serial_port_permission() {
    local serial_port="$1"
    
    # 检查串口是否存在
    if [ ! -e "$serial_port" ]; then
        print_warning "串口 $serial_port 未找到"
        print_info "可能的串口:"
        ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (未找到任何串口设备)"
        return 1
    fi
    
    # 检查串口权限
    if [ -r "$serial_port" ] && [ -w "$serial_port" ]; then
        print_success "串口 $serial_port 权限正常"
        return 0
    fi
    
    print_warning "串口 $serial_port 权限不足"
    print_info "尝试修复串口权限..."
    
    # 尝试使用 sudo 修改权限
    if sudo chmod 666 "$serial_port" 2>/dev/null; then
        print_success "串口权限已修复"
        return 0
    fi
    
    print_error "chmod 失败，请手动修改串口权限"
    return 1
}

# 检测ROS版本（ROS1或ROS2）
detect_ros_version() {
    # 优先检查ROS_VERSION环境变量
    if [ ! -z "$ROS_VERSION" ]; then
        return $ROS_VERSION
    fi
    
    # 检查ROS2
    if [ -f "/opt/ros/humble/setup.bash" ] || [ -f "/opt/ros/iron/setup.bash" ] || \
       [ -f "/opt/ros/rolling/setup.bash" ] || [ -f "/opt/ros/jazzy/setup.bash" ]; then
        return 2
    fi
    
    # 检查ROS1
    if [ -f "/opt/ros/noetic/setup.bash" ] || [ -f "/opt/ros/melodic/setup.bash" ] || \
       [ -f "/opt/ros/kinetic/setup.bash" ]; then
        return 1
    fi
    
    return 0
}

# 主函数
main() {
    print_info "qx_evk_driver 启动脚本"
    echo ""
    
    # 检查和修复串口权限
    local serial_port=$(get_serial_port)
    print_info "配置的串口: $serial_port"
    
    check_serial_port_permission "$serial_port"
    if [ $? -ne 0 ]; then
        print_error "串口权限检查失败"
        return 1
    fi
    
    echo ""
    
    # 检测ROS版本
    detect_ros_version
    ROS_VERSION=$?
    
    if [ $ROS_VERSION -eq 2 ]; then
        print_info "检测到ROS2环境，启动ROS2驱动..."
        echo ""
        
        # Source ROS2环境
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
        elif [ -f "/opt/ros/iron/setup.bash" ]; then
            source /opt/ros/iron/setup.bash
        elif [ -f "/opt/ros/rolling/setup.bash" ]; then
            source /opt/ros/rolling/setup.bash
        elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
            source /opt/ros/jazzy/setup.bash
        else
            print_error "未找到ROS2环境"
            return 1
        fi
        
        # Source本地编译的包
        if [ -f "${SCRIPT_DIR}/../install/setup.bash" ]; then
            source ${SCRIPT_DIR}/../install/setup.bash
        else
            print_error "未找到ROS2工作空间，请先执行: ./scripts/build.sh"
            return 1
        fi
        
        print_success "环境加载完成，启动驱动程序..."
        echo ""
        
        # 运行ROS2驱动程序
        ros2 launch qx_evk_driver qx_evk_driver.py
        
    elif [ $ROS_VERSION -eq 1 ]; then
        print_info "检测到ROS1环境，启动ROS1驱动..."
        echo ""
        
        # Source ROS1环境
        if [ -f "/opt/ros/noetic/setup.bash" ]; then
            source /opt/ros/noetic/setup.bash
        elif [ -f "/opt/ros/melodic/setup.bash" ]; then
            source /opt/ros/melodic/setup.bash
        elif [ -f "/opt/ros/kinetic/setup.bash" ]; then
            source /opt/ros/kinetic/setup.bash
        else
            print_error "未找到ROS1环境"
            return 1
        fi
        
        # Source catkin工作空间
        if [ -f "${SCRIPT_DIR}/../devel/setup.bash" ]; then
            source ${SCRIPT_DIR}/../devel/setup.bash
        else
            print_error "未找到ROS1工作空间，请先执行: ./scripts/build.sh"
            return 1
        fi
        
        print_success "环境加载完成，启动驱动程序..."
        echo ""
        
        # 运行ROS1驱动程序
        roslaunch qx_evk_driver qx_evk_driver.launch
        
    else
        print_error "未检测到ROS环境"
        print_info "请先安装并加载 ROS1 (Noetic/Melodic/Kinetic) 或 ROS2 (Humble/Iron/Rolling/Jazzy)"
        print_info "ROS2 安装示例: source /opt/ros/humble/setup.bash"
        print_info "ROS1 安装示例: source /opt/ros/noetic/setup.bash"
        return 1
    fi
}

# 运行主函数
main
exit $?