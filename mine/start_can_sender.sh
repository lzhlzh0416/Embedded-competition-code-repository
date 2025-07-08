#!/bin/bash

# 日志文件路径
LOG_FILE="/home/sunrise/can_sender.log"

# 记录脚本开始执行
echo "[$(date)] CAN 启动脚本开始执行" >> $LOG_FILE

# 1. 加载 ROS 2 环境
echo "[$(date)] 加载 ROS 2 环境..." >> $LOG_FILE
source /opt/ros/humble/setup.bash
if [ $? -ne 0 ]; then
    echo "[$(date)] 错误：无法加载 ROS 2 Humble 环境" >> $LOG_FILE
    exit 1
fi

source /home/sunrise/ros2_nav/rplidar_ws/install/setup.bash
if [ $? -ne 0 ]; then
    echo "[$(date)] 错误：无法加载工作空间环境" >> $LOG_FILE
    exit 1
fi

# 2. 配置 CAN 接口
echo "[$(date)] 配置 CAN 接口..." >> $LOG_FILE
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 500000 sample-point 0.875
sudo ip link set can1 up

# 检查 CAN 接口是否成功启动
CAN_STATUS=$(ip link show can1 | grep "state UP")
if [ -z "$CAN_STATUS" ]; then
    echo "[$(date)] 错误：CAN 接口未能成功启动" >> $LOG_FILE
    exit 1
else
    echo "[$(date)] CAN 接口已成功启动" >> $LOG_FILE
fi

# 3. 启动 ROS 2 节点（后台运行）
echo "[$(date)] 启动 ROS 2 节点..." >> $LOG_FILE
ros2 run can_bridge_cpp can_sender >> $LOG_FILE 2>&1 &

# 记录脚本执行完成
echo "[$(date)] CAN 启动脚本执行完成" >> $LOG_FILE