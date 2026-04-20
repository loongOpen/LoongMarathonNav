#!/bin/bash

# qx_evk_driver 环境设置脚本
# 自动检测和安装所需依赖

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
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

print_header() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}========================================${NC}"
}

# 显示使用说明
show_usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help          显示此帮助信息"
    echo "  -y, --yes           自动确认所有安装(非交互模式)"
    echo "  -c, --check-only    仅检查依赖，不安装"
    echo "  -r, --ros-only      仅检查/设置ROS环境"
    echo ""
    echo "示例:"
    echo "  $0                  # 交互式检查和安装依赖"
    echo "  $0 -y               # 自动安装所有缺失的依赖"
    echo "  $0 -c               # 仅检查依赖状态"
    echo "  $0 -r               # 仅检查ROS环境"
}

# 解析命令行参数
AUTO_YES=0
CHECK_ONLY=0
ROS_ONLY=0

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -y|--yes)
            AUTO_YES=1
            shift
            ;;
        -c|--check-only)
            CHECK_ONLY=1
            shift
            ;;
        -r|--ros-only)
            ROS_ONLY=1
            shift
            ;;
        *)
            print_error "未知选项: $1"
            show_usage
            exit 1
            ;;
    esac
done

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then
    print_warning "不建议以root用户运行此脚本"
    if [ $AUTO_YES -eq 0 ]; then
        read -p "是否继续？(y/N): " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# 检测操作系统
detect_os() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$NAME
        VER=$VERSION_ID
        print_info "检测到操作系统: $OS $VER"
    else
        print_error "无法检测操作系统版本"
        exit 1
    fi
}

# 检查ROS环境
check_ros() {
    print_header "检查ROS环境"
    
    local ros_found=0
    local ros_version=""
    local ros_distro=""
    
    # 检查环境变量
    if [ -n "$ROS_VERSION" ]; then
        ros_found=1
        ros_version=$ROS_VERSION
        ros_distro=$ROS_DISTRO
        
        if [ "$ROS_VERSION" = "1" ]; then
            print_success "ROS1 环境已加载 (发行版: $ROS_DISTRO)"
        elif [ "$ROS_VERSION" = "2" ]; then
            print_success "ROS2 环境已加载 (发行版: $ROS_DISTRO)"
        fi
    else
        print_warning "ROS环境未加载"
        
        # 尝试检测已安装的ROS版本
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            print_info "检测到 ROS2 Humble"
            ros_distro="humble"
            ros_version="2"
        elif [ -f "/opt/ros/foxy/setup.bash" ]; then
            print_info "检测到 ROS2 Foxy"
            ros_distro="foxy"
            ros_version="2"
        elif [ -f "/opt/ros/galactic/setup.bash" ]; then
            print_info "检测到 ROS2 Galactic"
            ros_distro="galactic"
            ros_version="2"
        elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
            print_info "检测到 ROS2 Jazzy"
            ros_distro="jazzy"
            ros_version="2"
        elif [ -f "/opt/ros/iron/setup.bash" ]; then
            print_info "检测到 ROS2 Iron"
            ros_distro="iron"
            ros_version="2"
        elif [ -f "/opt/ros/rolling/setup.bash" ]; then
            print_info "检测到 ROS2 Rolling"
            ros_distro="rolling"
            ros_version="2"
        elif [ -f "/opt/ros/noetic/setup.bash" ]; then
            print_info "检测到 ROS1 Noetic"
            ros_distro="noetic"
            ros_version="1"
        elif [ -f "/opt/ros/melodic/setup.bash" ]; then
            print_info "检测到 ROS1 Melodic"
            ros_distro="melodic"
            ros_version="1"
        elif [ -f "/opt/ros/kinetic/setup.bash" ]; then
            print_info "检测到 ROS1 Kinetic"
            ros_distro="kinetic"
            ros_version="1"
        else
            print_error "未检测到已安装的ROS"
            echo ""
            print_info "ROS2 安装指南:"
            echo "  Ubuntu 24.04: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html"
            echo "  Ubuntu 22.04: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
            echo "  Ubuntu 20.04: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html"
            echo ""
            print_info "ROS1 安装指南:"
            echo "  Ubuntu 20.04: http://wiki.ros.org/noetic/Installation/Ubuntu"
            echo "  Ubuntu 18.04: http://wiki.ros.org/melodic/Installation/Ubuntu"
            echo "  Ubuntu 16.04: http://wiki.ros.org/kinetic/Installation/Ubuntu"
            return 1
        fi
        
        # 提示用户source ROS环境
        echo ""
        print_info "请先加载ROS环境，然后重新运行此脚本:"
        echo "  source /opt/ros/$ros_distro/setup.bash"
        echo ""
        print_info "或者将以下命令添加到 ~/.bashrc 以自动加载:"
        echo "  echo 'source /opt/ros/$ros_distro/setup.bash' >> ~/.bashrc"
        
        if [ $CHECK_ONLY -eq 0 ] && [ $AUTO_YES -eq 0 ]; then
            echo ""
            read -p "是否现在自动添加到 ~/.bashrc？(y/N): " -n 1 -r
            echo ""
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                if ! grep -q "source /opt/ros/$ros_distro/setup.bash" ~/.bashrc; then
                    echo "source /opt/ros/$ros_distro/setup.bash" >> ~/.bashrc
                    print_success "已添加到 ~/.bashrc"
                    print_info "请运行: source ~/.bashrc"
                else
                    print_info "~/.bashrc 中已存在此配置"
                fi
            fi
        fi
        
        return 1
    fi
    
    return 0
}

# 检查系统依赖
check_system_dependencies() {
    print_header "检查系统依赖"
    
    local missing_deps=()
    local installed_deps=()
    
    # 检查Boost
    print_info "检查 Boost 库..."
    if dpkg -l 2>/dev/null | grep -q "^ii.*libboost-all-dev\|^ii.*libboost-dev"; then
        local boost_version=$(dpkg -l | grep -E "^ii.*libboost-dev" | awk '{print $3}' | head -n1)
        print_success "Boost 已安装 ($boost_version)"
        installed_deps+=("boost")
    else
        print_warning "Boost 未安装"
        missing_deps+=("libboost-all-dev")
    fi
    
    # 检查GeographicLib
    print_info "检查 GeographicLib..."
    if dpkg -l 2>/dev/null | grep -q "^ii.*libgeographic-dev"; then
        local geo_version=$(dpkg -l | grep "^ii.*libgeographic-dev" | awk '{print $3}')
        print_success "GeographicLib 已安装 ($geo_version)"
        installed_deps+=("geographiclib")
    else
        print_warning "GeographicLib 未安装"
        missing_deps+=("libgeographic-dev")
    fi
    
    # 检查CMake
    print_info "检查 CMake..."
    if command -v cmake &> /dev/null; then
        local cmake_version=$(cmake --version | head -n1 | awk '{print $3}')
        print_success "CMake 已安装 ($cmake_version)"
        installed_deps+=("cmake")
    else
        print_warning "CMake 未安装"
        missing_deps+=("cmake")
    fi
    
    # 检查编译工具
    print_info "检查编译工具..."
    if command -v g++ &> /dev/null; then
        local gcc_version=$(g++ --version | head -n1 | awk '{print $3}')
        print_success "g++ 已安装 ($gcc_version)"
        installed_deps+=("g++")
    else
        print_warning "g++ 未安装"
        missing_deps+=("build-essential")
    fi
    
    # 检查Git
    print_info "检查 Git..."
    if command -v git &> /dev/null; then
        local git_version=$(git --version | awk '{print $3}')
        print_success "Git 已安装 ($git_version)"
        installed_deps+=("git")
    else
        print_warning "Git 未安装"
        missing_deps+=("git")
    fi
    
    echo ""
    
    # 显示结果
    if [ ${#missing_deps[@]} -eq 0 ]; then
        print_success "所有系统依赖已满足 ✓"
        return 0
    else
        print_warning "缺失 ${#missing_deps[@]} 个依赖包:"
        for dep in "${missing_deps[@]}"; do
            echo "  - $dep"
        done
        
        if [ $CHECK_ONLY -eq 1 ]; then
            return 1
        fi
        
        echo ""
        
        # 询问是否安装
        local install_confirm=0
        if [ $AUTO_YES -eq 1 ]; then
            install_confirm=1
        else
            read -p "是否现在安装缺失的依赖？(y/N): " -n 1 -r
            echo ""
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                install_confirm=1
            fi
        fi
        
        if [ $install_confirm -eq 1 ]; then
            print_info "开始安装依赖..."
            
            # 更新包列表
            print_info "更新软件包列表..."
            sudo apt update
            
            # 安装依赖
            print_info "安装依赖包: ${missing_deps[*]}"
            sudo apt install -y "${missing_deps[@]}"
            
            print_success "依赖安装完成 ✓"
            return 0
        else
            print_warning "跳过安装，某些功能可能无法使用"
            return 1
        fi
    fi
}

# 检查ROS依赖包
check_ros_dependencies() {
    if [ -z "$ROS_VERSION" ]; then
        print_warning "跳过ROS依赖检查(ROS环境未加载)"
        return 0
    fi
    
    print_header "检查ROS依赖包"
    
    local missing_ros_deps=()
    
    if [ "$ROS_VERSION" = "2" ]; then
        # ROS2 依赖
        local ros2_deps=("rclcpp" "std_msgs" "geometry_msgs" "sensor_msgs" "ament_cmake")
        
        for dep in "${ros2_deps[@]}"; do
            print_info "检查 $dep..."
            if dpkg -l 2>/dev/null | grep -q "^ii.*ros-$ROS_DISTRO-${dep//_/-}"; then
                print_success "$dep 已安装"
            else
                print_warning "$dep 未安装"
                missing_ros_deps+=("ros-$ROS_DISTRO-${dep//_/-}")
            fi
        done
    elif [ "$ROS_VERSION" = "1" ]; then
        # ROS1 依赖
        local ros1_deps=("roscpp" "std_msgs" "geometry_msgs" "sensor_msgs" "roslib")
        
        for dep in "${ros1_deps[@]}"; do
            print_info "检查 $dep..."
            if dpkg -l 2>/dev/null | grep -q "^ii.*ros-$ROS_DISTRO-${dep//_/-}"; then
                print_success "$dep 已安装"
            else
                print_warning "$dep 未安装"
                missing_ros_deps+=("ros-$ROS_DISTRO-${dep//_/-}")
            fi
        done
        
        # 提示使用 rosdep
        echo ""
        print_info "提示: 您也可以使用 rosdep 自动安装所有依赖:"
        echo "  rosdep install --from-paths . --ignore-src -r -y"
    fi
    
    echo ""
    
    if [ ${#missing_ros_deps[@]} -eq 0 ]; then
        print_success "所有ROS依赖已满足 ✓"
        return 0
    elif [ "$ROS_VERSION" = "1" ] && command -v rosdep &> /dev/null; then
        print_info "ROS1检测到rosdep工具，优先使用rosdep安装依赖"
        echo ""
        
        if [ $CHECK_ONLY -eq 1 ]; then
            print_warning "缺失ROS依赖，请运行: rosdep install --from-paths . --ignore-src -r -y"
            return 1
        fi
        
        local install_confirm=0
        if [ $AUTO_YES -eq 1 ]; then
            install_confirm=1
        else
            read -p "是否使用rosdep安装ROS依赖？(y/N): " -n 1 -r
            echo ""
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                install_confirm=1
            fi
        fi
        
        if [ $install_confirm -eq 1 ]; then
            print_info "使用rosdep安装依赖..."
            rosdep install --from-paths . --ignore-src -r -y
            print_success "ROS依赖安装完成 ✓"
            return 0
        else
            print_warning "跳过安装，编译可能失败"
            return 1
        fi
    else
        print_warning "缺失 ${#missing_ros_deps[@]} 个ROS依赖包:"
        for dep in "${missing_ros_deps[@]}"; do
            echo "  - $dep"
        done
        
        if [ $CHECK_ONLY -eq 1 ]; then
            return 1
        fi
        
        echo ""
        
        # 询问是否安装
        local install_confirm=0
        if [ $AUTO_YES -eq 1 ]; then
            install_confirm=1
        else
            read -p "是否现在安装缺失的ROS依赖？(y/N): " -n 1 -r
            echo ""
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                install_confirm=1
            fi
        fi
        
        if [ $install_confirm -eq 1 ]; then
            print_info "开始安装ROS依赖..."
            sudo apt update
            sudo apt install -y "${missing_ros_deps[@]}"
            print_success "ROS依赖安装完成 ✓"
            return 0
        else
            print_warning "跳过安装，编译可能失败"
            return 1
        fi
    fi
}

# 检查工作空间设置
check_workspace() {
    print_header "检查工作空间"
    
    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local project_dir="$(cd "$script_dir/.." && pwd)"
    
    print_info "项目目录: $project_dir"
    
    # 检查必要的文件
    local required_files=("CMakeLists.txt" "package.xml")
    local missing_files=0
    
    for file in "${required_files[@]}"; do
        if [ -f "$project_dir/$file" ]; then
            print_success "找到 $file"
        else
            print_error "缺少 $file"
            missing_files=1
        fi
    done
    
    if [ $missing_files -eq 1 ]; then
        print_error "项目文件不完整"
        return 1
    fi
    
    echo ""
    print_success "工作空间检查完成 ✓"
    return 0
}

# 显示环境信息
show_environment_info() {
    print_header "环境信息摘要"
    
    echo "操作系统: $OS $VER"
    
    if [ -n "$ROS_VERSION" ]; then
        echo "ROS版本: ROS$ROS_VERSION ($ROS_DISTRO)"
    else
        echo "ROS版本: 未加载"
    fi
    
    if command -v cmake &> /dev/null; then
        echo "CMake: $(cmake --version | head -n1 | awk '{print $3}')"
    fi
    
    if command -v g++ &> /dev/null; then
        echo "GCC: $(g++ --version | head -n1 | awk '{print $3}')"
    fi
    
    echo ""
}

# 显示下一步操作
show_next_steps() {
    print_header "下一步操作"
    
    if [ -z "$ROS_VERSION" ]; then
        echo "1. 选择并加载ROS环境:"
        echo ""
        echo "   ROS2 (推荐):"
        echo "     source /opt/ros/humble/setup.bash  # Ubuntu 22.04+"
        echo "     或"
        echo "     source /opt/ros/jazzy/setup.bash   # Ubuntu 24.04+"
        echo ""
        echo "   ROS1:"
        echo "     source /opt/ros/noetic/setup.bash  # Ubuntu 20.04"
        echo "     或"
        echo "     source /opt/ros/melodic/setup.bash # Ubuntu 18.04"
        echo "     或"
        echo "     source /opt/ros/kinetic/setup.bash # Ubuntu 16.04"
        echo ""
        echo "2. 重新运行此脚本:"
        echo "   ./scripts/setup.sh"
    else
        echo "1. 编译项目:"
        echo "   ./scripts/build.sh"
        echo ""
        echo "2. 运行项目:"
        if [ "$ROS_VERSION" = "2" ]; then
            echo "   自动检测版本运行(推荐):"
            echo "     ./scripts/run.sh"
            echo ""
            echo "   或手动运行:"
            echo "     source install/setup.bash"
            echo "     ros2 launch qx_evk_driver qx_evk_driver.py"
        else
            echo "   自动检测版本运行(推荐):"
            echo "     ./scripts/run.sh"
            echo ""
            echo "   或手动运行:"
            echo "     source devel/setup.bash"
            echo "     roslaunch qx_evk_driver qx_evk_driver.launch"
        fi
    fi
    
    echo ""
}

# 主函数
main() {
    print_header "qx_evk_driver 环境设置"
    echo ""
    
    # 检测操作系统
    detect_os
    echo ""
    
    # 检查ROS环境
    check_ros
    local ros_status=$?
    echo ""
    
    # 如果仅检查ROS，则退出
    if [ $ROS_ONLY -eq 1 ]; then
        if [ $ros_status -eq 0 ]; then
            print_success "ROS环境正常 ✓"
            exit 0
        else
            print_error "ROS环境未就绪"
            exit 1
        fi
    fi
    
    # 检查系统依赖
    check_system_dependencies
    local sys_deps_status=$?
    echo ""
    
    # 检查ROS依赖
    if [ $ros_status -eq 0 ]; then
        check_ros_dependencies
        echo ""
    fi
    
    # 检查工作空间
    check_workspace
    echo ""
    
    # 显示环境信息
    show_environment_info
    
    # 显示下一步操作
    show_next_steps
    
    # 返回状态
    if [ $ros_status -eq 0 ] && [ $sys_deps_status -eq 0 ]; then
        print_success "环境设置完成！✓"
        exit 0
    else
        print_warning "环境设置完成，但存在一些问题"
        exit 1
    fi
}

# 运行主函数
main
