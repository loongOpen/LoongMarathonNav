#!/bin/bash

# 串口权限修复脚本
# 用于快速修复串口权限问题

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

# 显示使用说明
show_usage() {
    echo "用法: $0 [串口设备] [选项]"
    echo ""
    echo "串口设备:"
    echo "  /dev/ttyUSB0      # USB转串口（默认）"
    echo "  /dev/ttyACM0      # Arduino USB设备"
    echo "  /dev/ttyS0        # 传统串口"
    echo ""
    echo "选项:"
    echo "  -h, --help        显示此帮助信息"
    echo "  -l, --list        列出所有可用的串口设备"
    echo "  -a, --auto        自动检测并修复所有串口"
    echo ""
    echo "示例:"
    echo "  $0                         # 修复默认串口"
    echo "  $0 /dev/ttyUSB0            # 修复指定串口"
    echo "  $0 -l                      # 列出所有串口"
    echo "  $0 -a                      # 自动修复所有串口"
}

# 列出所有可用的串口
list_serial_ports() {
    print_info "扫描可用的串口设备..."
    echo ""
    
    local found=0
    for port in /dev/ttyUSB* /dev/ttyACM* /dev/ttyS* 2>/dev/null; do
        if [ -e "$port" ]; then
            found=1
            local perms=$(ls -l "$port" | awk '{print $1}')
            local owner=$(ls -l "$port" | awk '{print $3":"$4}')
            echo "  $port ($perms, $owner)"
        fi
    done
    
    if [ $found -eq 0 ]; then
        print_warning "未找到任何串口设备"
        echo "请检查:"
        echo "  1. 设备是否正确连接"
        echo "  2. 驱动程序是否已安装"
        echo "  3. 使用 lsusb 或 dmesg 检查设备信息"
        return 1
    fi
    
    echo ""
}

# 修复单个串口权限
fix_port_permission() {
    local port=$1
    
    if [ -z "$port" ]; then
        port="/dev/ttyUSB0"
    fi
    
    print_info "处理串口: $port"
    
    # 检查串口是否存在
    if [ ! -e "$port" ]; then
        print_error "串口 $port 不存在"
        echo ""
        print_info "使用 $0 -l 查看可用的串口"
        return 1
    fi
    
    # 检查权限
    if [ -r "$port" ] && [ -w "$port" ]; then
        print_success "串口 $port 权限正常"
        return 0
    fi
    
    print_warning "串口 $port 权限不足"
    echo ""
    
    # 方案1: 使用 chmod
    print_info "方案1: 使用 chmod 修改权限..."
    if sudo chmod 666 "$port" 2>/dev/null; then
        print_success "权限修改成功: chmod 666 $port"
        return 0
    fi
    
    print_warning "chmod 修改失败，尝试方案2..."
    echo ""
    
    # 方案2: 添加用户到 dialout 组
    print_info "方案2: 将用户添加到 dialout 组..."
    
    local current_user=$(whoami)
    if id -nG "$current_user" | grep -qw "dialout"; then
        print_warning "用户 $current_user 已在 dialout 组中"
        echo "请尝试:"
        echo "  1. 使用 sudo 运行驱动: sudo ./scripts/run.sh"
        echo "  2. 或手动修改权限: sudo chmod 666 $port"
        return 1
    fi
    
    print_info "将用户 $current_user 添加到 dialout 组..."
    if sudo usermod -aG dialout "$current_user" 2>/dev/null; then
        print_success "用户已添加到 dialout 组"
        echo ""
        print_warning "请注销并重新登录以应用该更改"
        echo ""
        print_info "快速应用方法:"
        echo "  运行: newgrp dialout"
        echo "  然后: ./scripts/run.sh"
        return 0
    fi
    
    print_error "添加用户到 dialout 组失败"
    echo ""
    print_info "请尝试手动修改:"
    echo "  sudo chmod 666 $port"
    return 1
}

# 自动修复所有串口
auto_fix_all_ports() {
    print_info "自动修复所有串口..."
    echo ""
    
    local ports=(/dev/ttyUSB* /dev/ttyACM* /dev/ttyS* 2>/dev/null)
    
    if [ ${#ports[@]} -eq 0 ]; then
        print_warning "未找到任何串口设备"
        return 1
    fi
    
    for port in "${ports[@]}"; do
        if [ -e "$port" ]; then
            fix_port_permission "$port"
            echo ""
        fi
    done
}

# 主函数
main() {
    print_info "串口权限修复工具"
    echo ""
    
    # 检查是否为 root 用户
    if [ "$EUID" -eq 0 ]; then
        print_warning "以 root 用户身份运行此脚本"
        echo ""
    fi
    
    # 解析命令行参数
    case "${1:-}" in
        -h|--help)
            show_usage
            exit 0
            ;;
        -l|--list)
            list_serial_ports
            exit 0
            ;;
        -a|--auto)
            auto_fix_all_ports
            exit $?
            ;;
        "")
            # 修复默认串口
            fix_port_permission "/dev/ttyUSB0"
            exit $?
            ;;
        *)
            # 修复指定串口
            fix_port_permission "$1"
            exit $?
            ;;
    esac
}

# 运行主函数
main "$@"
