#!/bin/bash

# qx_evk_driver 编译脚本
# 支持ROS1和ROS2自动检测和编译

set -e  # 遇到错误立即退出

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
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示使用说明
show_usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help          显示此帮助信息"
    echo "  -c, --clean         清理编译文件后重新编译"
    echo "  -r, --release       Release模式编译 (默认)"
    echo "  -d, --debug         Debug模式编译"
    echo "  -j N                使用N个并行任务编译 (默认: CPU核心数)"
    echo ""
    echo "示例:"
    echo "  $0                  # 正常编译"
    echo "  $0 -c               # 清理后重新编译"
    echo "  $0 -l               # 编译到本地目录"
    echo "  $0 -c -l -d         # 清理、本地、Debug模式"
}


# 检测ROS版本
detect_ros_version() {
    if [ -n "$ROS_VERSION" ]; then
        if [ "$ROS_VERSION" = "1" ]; then
            print_info "检测到 ROS1 环境"
            if [ -n "$ROS_DISTRO" ]; then
                print_info "ROS发行版: $ROS_DISTRO"
            fi
            echo "1"
        elif [ "$ROS_VERSION" = "2" ]; then
            print_info "检测到 ROS2 环境"
            if [ -n "$ROS_DISTRO" ]; then
                print_info "ROS发行版: $ROS_DISTRO"
            fi
            echo "2"
        fi
    else
        print_error "未检测到ROS环境！"
        echo "请先source ROS环境:"
        echo "  ROS1: source /opt/ros/noetic/setup.bash"
        echo "  ROS2: source /opt/ros/humble/setup.bash"
        exit 1
    fi
}

# 清理编译文件
clean_build() {
    print_info "清理编译文件..."
    
    # 清理ROS1编译文件
    if [ -d "build" ] || [ -d "devel" ]; then
        rm -rf build/ devel/ install/
        print_success "ROS1编译文件已清理"
    fi
    
    # 清理ROS2编译文件
    if [ -d "build" ] || [ -d "install" ]; then
        rm -rf build/ install/ log/
        print_success "ROS2编译文件已清理"
    fi
    
    if [ ! -d "build" ] && [ ! -d "devel" ] && [ ! -d "install" ]; then
        print_success "所有编译文件已清理"
    fi
}

# ROS1编译
build_ros1() {
    print_info "开始ROS1编译..."
    
    # 检查ROS1环境
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS1环境未正确加载"
        echo "请先source ROS1环境:"
        echo "  source /opt/ros/noetic/setup.bash  # 或 melodic/kinetic"
        exit 1
    fi
    
    print_info "ROS1发行版: $ROS_DISTRO"
    
    # 进入工作空间根目录
    cd ..
    
    # 如果存在之前的编译结果，进行清理提示
    if [ "$CLEAN" = true ] && [ -d "devel" ]; then
        print_info "清理ROS1编译文件..."
        rm -rf build/ devel/ install/
        print_success "ROS1编译文件已清理"
    fi
    
    # 检查CMakeLists.txt
    if [ ! -f "CMakeLists.txt" ]; then
        print_error "未找到CMakeLists.txt，请确保在项目根目录"
        exit 1
    fi
    
    # 执行catkin_make
    print_info "执行catkin_make (BUILD_TYPE=$BUILD_TYPE, JOBS=$JOBS)..."
    if [ "$BUILD_TYPE" = "Debug" ]; then
        catkin_make -DCMAKE_BUILD_TYPE=Debug -j${JOBS}
    else
        catkin_make -DCMAKE_BUILD_TYPE=Release -j${JOBS}
    fi
    
    if [ $? -eq 0 ]; then
        print_success "ROS1编译成功！"
        echo ""
        print_info "运行方法:"
        echo "  1. Source ROS1环境:"
        echo "     source devel/setup.bash"
        echo "  2. 运行驱动程序:"
        echo "     roslaunch qx_evk_driver qx_evk_driver.launch"
        echo ""
        print_info "或使用运行脚本:"
        echo "  ./scripts/run.sh"
    else
        print_error "ROS1编译失败！"
        print_info "可能的解决方案:"
        echo "  1. 检查ROS1是否正确安装: which roslaunch"
        echo "  2. 检查依赖: rosdep install --from-paths . --ignore-src -r -y"
        echo "  3. 清理后重新编译: ./scripts/build.sh -c"
        exit 1
    fi
}
    
# ROS2编译
build_ros2() {
    print_info "开始ROS2编译..."
    
    # Source ROS2环境以获取colcon等工具
    if [ -z "$AMENT_PREFIX_PATH" ]; then
        print_info "加载ROS2环境..."
        source /opt/ros/humble/setup.bash
    fi
    
    # 强制使用Python 3.10
    local python_executable="/usr/bin/python3.10"
    
    if [ ! -f "$python_executable" ]; then
        print_error "未找到 Python 3.10"
        print_info "请安装: sudo apt install python3.10 python3.10-dev"
        exit 1
    fi
    
    print_info "使用Python: $python_executable ($($python_executable --version))"
    
    local ros_python_path="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages"
    local ros_local_path="/opt/ros/${ROS_DISTRO}/local/lib/python3.10/dist-packages"
    
    # 构建新的PYTHONPATH，只包含ROS2相关路径
    local new_pythonpath="${ros_python_path}"
    if [ -d "$ros_local_path" ]; then
        new_pythonpath="${new_pythonpath}:${ros_local_path}"
    fi
    
    export PYTHONPATH="$new_pythonpath"
    export Python3_ROOT_DIR="/usr"
    export Python3_EXECUTABLE="$python_executable"

    
    local colcon_args="--cmake-args"
    colcon_args="$colcon_args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    colcon_args="$colcon_args -DPython3_EXECUTABLE=${python_executable}"
    colcon_args="$colcon_args -DPython3_INCLUDE_DIR=/usr/include/python3.10"
    colcon_args="$colcon_args -DPython3_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so"
    colcon_args="$colcon_args -DPYTHON_EXECUTABLE=${python_executable}"
    
    print_info "PYTHONPATH: $PYTHONPATH"
    
    # 使用--symlink-install使路径可移植
    colcon build --install-base ./install --merge-install --symlink-install ${colcon_args} --parallel-workers ${JOBS}
    
    local build_result=$?
    
    # 编译成功后，修复setup脚本中的硬编码路径
    if [ $build_result -eq 0 ]; then
        print_info "修复路径引用，使安装包可移植..."
        
        # 修复setup.sh
        if [ -f "./install/setup.sh" ]; then
            sed -i "s|_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX=.*install|_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX=\$( cd \"\$( dirname \"\${BASH_SOURCE[0]:-\${(%):-%x}}\" )\" \&\& pwd )|g" ./install/setup.sh
        fi
        
        # 修复setup.bash
        if [ -f "./install/setup.bash" ]; then
            sed -i "s|_colcon_prefix_chain_bash_COLCON_CURRENT_PREFIX=.*install|_colcon_prefix_chain_bash_COLCON_CURRENT_PREFIX=\$( cd \"\$( dirname \"\${BASH_SOURCE[0]}\" )\" \&\& pwd )|g" ./install/setup.bash
        fi
        
        # 修复setup.zsh
        if [ -f "./install/setup.zsh" ]; then
            sed -i "s|_colcon_prefix_chain_zsh_COLCON_CURRENT_PREFIX=.*install|_colcon_prefix_chain_zsh_COLCON_CURRENT_PREFIX=\$( cd \"\$( dirname \"\${(%):-%x}\" )\" \&\& pwd )|g" ./install/setup.zsh
        fi
        
        print_success "路径修复完成，安装包现在可以在任意位置运行"
    fi
    
    if [ $build_result -eq 0 ]; then
        print_success "ROS2编译成功！"
        echo ""
        print_info "运行方法:"
        echo "  source install/setup.bash"
        echo "  ros2 launch qx_evk_driver qx_evk_driver.py"
    else
        print_error "编译失败！"
        exit 1
    fi
}

# 主函数
main() {
    # 默认参数
    CLEAN=false
    BUILD_TYPE="Release"
    JOBS=$(nproc)
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_usage
                exit 0
                ;;
            -c|--clean)
                CLEAN=true
                shift
                ;;
            -r|--release)
                BUILD_TYPE="Release"
                shift
                ;;
            -d|--debug)
                BUILD_TYPE="Debug"
                shift
                ;;
            -j)
                JOBS="$2"
                shift 2
                ;;
            *)
                print_error "未知选项: $1"
                show_usage
                exit 1
                ;;
        esac
    done
    
    echo "=========================================="
    echo "  qx_evk_driver 编译脚本"
    echo "=========================================="

    
    # 检测ROS版本
    ROS_VER=$(detect_ros_version)
    
    # 显示编译配置
    print_info "编译配置:"
    echo "  构建类型: $BUILD_TYPE"
    echo "  并行任务: $JOBS"
    
    # 清理
    if [ "$CLEAN" = true ]; then
        clean_build
    fi
    
    echo ""
    print_info "开始编译..."
    echo ""
    
    # 根据ROS版本编译
    if [ "$ROS_VER" = "1" ]; then
        build_ros1
    else
        build_ros2
    fi
    
    echo ""
    echo "=========================================="
    print_success "编译流程完成！"
    echo "=========================================="
}

# 运行主函数
main "$@"
