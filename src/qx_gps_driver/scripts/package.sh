#!/bin/bash

# 打包ROS2驱动程序脚本
# 用于将编译好的程序打包成可在其他环境运行的独立包

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
PACKAGE_NAME="qx_evk_driver_deploy_${TIMESTAMP}"
OUTPUT_DIR="output/${PACKAGE_NAME}"

echo "=========================================="
echo "开始打包 qx_evk_driver"
echo "=========================================="

# 创建打包目录
mkdir -p ${OUTPUT_DIR}

# 复制安装文件
echo "复制程序文件..."
cp -r install ${OUTPUT_DIR}/

# 复制脚本
cp -r scripts ${OUTPUT_DIR}/

# 复制配置文件
cp -r config ${OUTPUT_DIR}/

# 打包成tar.gz
echo "压缩打包..."
cd output
tar -czf ${PACKAGE_NAME}.tar.gz ${PACKAGE_NAME}

echo ""
echo "=========================================="
echo "打包完成！"
echo "=========================================="
echo "输出文件: ${PACKAGE_NAME}.tar.gz"
echo ""
echo "部署步骤："
echo "1. 将 ${PACKAGE_NAME}.tar.gz 复制到目标机器"
echo "2. 解压: tar -xzf ${PACKAGE_NAME}.tar.gz"
echo "3. 进入目录: cd ${PACKAGE_NAME}"
echo "4. 初始化环境: ./setup.sh"
echo "5. 运行程序: ./run.sh"
echo "=========================================="
