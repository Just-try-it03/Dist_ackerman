#pragma once

// 系统标准库头文件
#include <iostream>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include "jarvis_msgs/Clearance.h"
#include "jarvis_msgs/Cmd.h"
#include "jarvis_msgs/Info.h"
#include "jarvis_msgs/Json.h"
#include "jarvis_msgs/LocResult.h"
#include "jarvis_msgs/MotorAckerman.h"
#include "jarvis_msgs/OdomSteer.h"
#include "jarvis_msgs/RobotStatus.h"
#include "jarvis_msgs/VelAckerman.h"
#include "jarvis_msgs/VelSteer.h"

#include "common.h"
#include "mpc_controller_nmpc.h"
#include "ros_utils.h"
#include "zmq_server.h"

class MasterRosNode {
public:
    /**
     * @brief 构造函数
     * @param p_server ZMQ服务器指针
     * @param wheel_base 车辆轴距
     * @param track_width 车辆轮距
     */
    MasterRosNode(ZmqServer* p_server, double wheel_base, double track_width)
        : mp_server(p_server), L_(wheel_base), W_(track_width)
    {
        m_last_recv_cmd.cmd = -1;
        initMPCParams();

        // 订阅器初始化
        m_subers["motor_slave"] = m_nh.subscribe<jarvis_msgs::MotorAckerman>(
            "/jmotor_ackermann", 1, &MasterRosNode::SetMotorReal, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["jinfo"] = m_nh.subscribe<jarvis_msgs::Info>(
            "/jinfo", 1, &MasterRosNode::SetRobotInfo, this);
        m_subers["cmd_res"] = m_nh.subscribe(
            "/jcmd_res", 1000, &MasterRosNode::SetCmdRes, this,
            ros::TransportHints().tcpNoDelay());

        m_subers["master_laser_0"] = m_nh.subscribe<sensor_msgs::LaserScan>(
            "/robot_0/base_scan_0", 1, &MasterRosNode::MasterLaser0Callback, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["master_laser_1"] = m_nh.subscribe<sensor_msgs::LaserScan>(
            "/robot_0/base_scan_1", 1, &MasterRosNode::MasterLaser1Callback, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["master_odom"] = m_nh.subscribe<nav_msgs::Odometry>(
            "/robot_0/odom", 1, &MasterRosNode::MasterOdomCallback, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["global_path"] = m_nh.subscribe<nav_msgs::Path>(
            "/bz_robot/global_path", 1, &MasterRosNode::GlobalPathCallback, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["local_path"] = m_nh.subscribe<nav_msgs::Path>(
            "/bz_robot/local_path", 1, &MasterRosNode::LocalPathCallback, this,
            ros::TransportHints().tcpNoDelay());

        // 发布器初始化
        m_pubers["slave_laser"] = m_nh.advertise<sensor_msgs::LaserScan>("/slave_laser", 1);
        m_pubers["cmd_vel_0"] = m_nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);

        // 控制定时器：100ms周期（10Hz）
        m_control_timer = m_nh.createSteadyTimer(
            ros::WallDuration(0.1), &MasterRosNode::CycleControlCallback, this);
    }

    template <typename T>
    T clamp(T value, T min, T max)
    {
        value = std::max(value, min);
        value = std::min(value, max);
        return value;
    }

    template <typename T>
    static inline std::string serialize_ros(const T& ros_msg) {
        uint32_t size = ros::serialization::serializationLength(ros_msg);
        std::string buffer(size, 0);
        ros::serialization::OStream stream(reinterpret_cast<uint8_t*>(&buffer[0]), size);
        ros::serialization::serialize(stream, ros_msg);
        return buffer;
    }


    template <typename T>
    static inline T deserialize_ros(const std::string& buffer) {
        T msg_ros;
        ros::serialization::IStream stream(reinterpret_cast<uint8_t*>(buffer.data()), buffer.size());
        ros::serialization::deserialize(stream, msg_ros);
        return msg_ros;
    }

    // ==================== 系统回调函数 ====================
    /**
     * @brief 电机数据回调（从车）
     * @param msg 电机阿克曼消息指针
     */
    void SetMotorReal(const jarvis_msgs::MotorAckerman::ConstPtr& msg)
    {
        // mp_server->Send("", "motor_akm_slave", serialize_ros(*msg));
    }

    /**
     * @brief 机器人信息回调
     * @param msg 机器人信息消息指针
     */
    void SetRobotInfo(const jarvis_msgs::Info::ConstPtr& msg)
    {
        // mp_server->Send("", "info_slave", serialize_ros(*msg));
        // CheckCmd();
    }

    /**
     * @brief 设置机器人状态
     * @param msg 机器人状态消息
     */
    void SetRobotStatus(const jarvis_msgs::RobotStatus& msg)
    {
    }

    /**
     * @brief 设置从车电机数据
     * @param msg 电机阿克曼消息
     */
    void SetSlaveMotor(const jarvis_msgs::MotorAckerman& msg)
    {
    }

    /**
     * @brief 设置从车信息
     * @param msg 机器人信息消息
     */
    void SetSlaveInfo(const jarvis_msgs::Info& msg)
    {
    }

    /**
     * @brief 从车1里程计回调
     * @param msg 里程计消息
     */
    void SetSlaveOdom_1(const nav_msgs::Odometry& msg)
    {
        double x = msg.pose.pose.position.x;
        double y = msg.pose.pose.position.y;
        double yaw = tf::getYaw(msg.pose.pose.orientation);

        // 数据有效性校验
        if (std::isnan(x) || std::isnan(y) || std::isnan(yaw) ||
            fabs(x) > 1000 || fabs(y) > 1000) {
            PRINT_WARN("Slave1 odom数据异常，跳过更新！x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
            return;
        }

        slave_odom_1 = msg;
        have_slave_odom_1 = true;
    }

    void SetSlaveOdom_2(const nav_msgs::Odometry& msg)
    {
        double x = msg.pose.pose.position.x;
        double y = msg.pose.pose.position.y;
        double yaw = tf::getYaw(msg.pose.pose.orientation);

        // 数据有效性校验
        if (std::isnan(x) || std::isnan(y) || std::isnan(yaw) ||
            fabs(x) > 1000 || fabs(y) > 1000) {
            PRINT_WARN("Slave2 odom数据异常，跳过更新！x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
            return;
        }

        slave_odom_2 = msg;
        have_slave_odom_2 = true;
    }

    void SetSlaveLaser_0(const sensor_msgs::LaserScan& msg)
    {
        // std::lock_guard<std::mutex> lk(queue_mtx_);
        // PRINT_INFO("***SlaveLaser_0接收完成：%.2f",msg.angle_min);
    }

    /**
     * @brief 从车激光1数据回调
     * @param msg 激光扫描消息
     */
    void SetSlaveLaser_1(const sensor_msgs::LaserScan& msg)
    {
        // std::lock_guard<std::mutex> lk(queue_mtx_);
        // PRINT_INFO("***SlaveLaser_1接收完成：%.2f",msg.angle_min);
    }

    /**
     * @brief 指令响应回调
     * @param cmd 指令响应消息
     */
    void SetCmdRes(const jarvis_msgs::Cmd cmd)
    {
        m_last_recv_cmd = cmd;
        CheckCmd();
        m_last_recv_cmd.cmd = -1;
    }

    /**
     * @brief 主车激光1数据回调
     * @param laser 激光扫描消息指针
     */
    void MasterLaser1Callback(const sensor_msgs::LaserScan::ConstPtr& laser)
    {
        try {
            if (!laser || laser->ranges.empty()) {
                PRINT_WARN("Received empty laser data");
                return;
            }

            Json j1;
            j1["data"] = zbase64::encode(serialize_ros(*laser));
            mp_server->Send("client-reliable", "masterLaser1", j1.dump());
        } catch (const std::exception& e) {
            PRINT_ERROR("Error processing master laser data: %s", e.what());
        }
    }

    /**
     * @brief 主车激光0数据回调
     * @param laser 激光扫描消息指针
     */
    void MasterLaser0Callback(const sensor_msgs::LaserScan::ConstPtr& laser)
    {
        try {
            if (!laser || laser->ranges.empty()) {
                PRINT_WARN("Received empty laser data");
                return;
            }

            // 创建消息副本并更新时间戳
            sensor_msgs::LaserScan laser_copy = *laser;
            laser_copy.header.stamp = ros::Time::now();

            Json j0;
            j0["data"] = zbase64::encode(serialize_ros(laser_copy));
            mp_server->Send("client-reliable", "masterLaser0", j0.dump());
        } catch (const std::exception& e) {
            PRINT_ERROR("Error processing master laser data: %s", e.what());
        }
    }

    /**
     * @brief 主车里程计回调
     * @param msg 里程计消息指针
     */
    void MasterOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double yaw = tf::getYaw(msg->pose.pose.orientation);

        // 数据有效性校验
        if (std::isnan(x) || std::isnan(y) || std::isnan(yaw) ||
            fabs(x) > 1000 || fabs(y) > 1000) {
            PRINT_WARN("Master odom数据异常，跳过更新！x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
            return;
        }

        master_odom_ = *msg;
        have_master_odom_ = true;
    }

    /**
     * @brief 全局路径回调
     * @param msg 路径消息指针
     */
    void GlobalPathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
    }

    void LocalPathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        std::vector<SimpleFormationNMPCController::RefPoint> ref;
        ref.reserve(msg->poses.size());

        for (const auto& pose_stamped : msg->poses) {
            SimpleFormationNMPCController::RefPoint p;
            p.x = pose_stamped.pose.position.x;
            p.y = pose_stamped.pose.position.y;

            // 解析姿态角
            double roll, pitch, yaw;
            tf::Matrix3x3(
                tf::Quaternion(
                    pose_stamped.pose.orientation.x,
                    pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z,
                    pose_stamped.pose.orientation.w
                )
            ).getRPY(roll, pitch, yaw);

            p.yaw = yaw;
            p.v = 0.6;  // 期望速度（常数）
            ref.push_back(p);
        }

        // 设置MPC参考路径并通过ZMQ发送
        master_mpc_.setReferencePath(ref);
        Json j;
        j["data"] = zbase64::encode(serialize_ros(ref));
        mp_server->Send("client-reliable", "local_path_ref", j.dump());
        mp_server->Send("client-reliable1", "local_path_ref", j.dump());
    }

    // ==================== 控制与指令处理 ====================
    void CycleControlCallback(const ros::SteadyTimerEvent& event)
    {
        PRINT_INFO("主车控制周期回调开始执行");

        if (!have_master_odom_) {
            PRINT_WARN("缺少master odom数据，跳过控制周期");
            return;
        }

        geometry_msgs::Twist master_twist = computeMasterPathFollowing();

        PublishTwist(master_twist);

        PRINT_INFO("准备发送编队参考信息给两辆从车");
        sendFormationReference(master_twist);

        logMasterControlStatus(master_twist);
        logMasterPosition();
    }

    void PublishTwist(const geometry_msgs::Twist& master_twist)
    {
        auto it = m_pubers.find("cmd_vel_0");
        if (it != m_pubers.end() && it->second) {
            it->second.publish(master_twist);
        }
    }

    void PubCmd(const int8_t cmd, const int8_t arg8, const int32_t arg32, const std::string& str)
    {
        jarvis_msgs::Cmd msg;
        msg.cmd = cmd;
        msg.data.arg_int8 = arg8;
        msg.data.arg_int32 = arg32;
        msg.data.arg_str = str;

#if 0
        pubers_["cmd"].publish(msg);
        PRINT_INFO("publish cmd: [%d %d %d %s]", cmd, arg8, arg32, str.c_str());
#endif

        m_last_recv_cmd.cmd = -1;
        m_send_cmd_queue.push_back(msg);
        CheckCmd();
        m_last_recv_cmd.cmd = -1;
    }

    /**
     * @brief 检查指令执行状态
     */
    void CheckCmd()
    {
        const auto& cmd = m_last_recv_cmd;
        if (!m_send_cmd_queue.empty()) {
            const auto& sent_cmd = m_send_cmd_queue.front();
            if (cmd.cmd == sent_cmd.cmd &&
                cmd.data.arg_int8 == sent_cmd.data.arg_int8 &&
                cmd.data.arg_int32 == sent_cmd.data.arg_int32) {
                m_send_cmd_queue.pop_front();
                PRINT_INFO("publish cmd success");
            }
        }
    }

    // ==================== 跟随控制算法 ====================
    geometry_msgs::Twist computeMasterPathFollowing()
    {
        std::vector<double> pose(3);
        pose[0] = master_odom_.pose.pose.position.x;
        pose[1] = master_odom_.pose.pose.position.y;

        double roll, pitch, yaw;
        tf::Matrix3x3(
            tf::Quaternion(
                master_odom_.pose.pose.orientation.x,
                master_odom_.pose.pose.orientation.y,
                master_odom_.pose.pose.orientation.z,
                master_odom_.pose.pose.orientation.w
            )
        ).getRPY(roll, pitch, yaw);
        pose[2] = yaw;

        std::vector<double> cmd = master_mpc_.computeControl(pose);

        geometry_msgs::Twist twist;
        twist.linear.x = cmd[0];
        twist.angular.z = cmd[1];
        return twist;
    }

    void sendFormationReference(const geometry_msgs::Twist& master_twist)
    {
        if (!mp_server) {
            PRINT_ERROR("ZMQ服务器未初始化，无法发送消息");
            return;
        }

        // 构造参考信息
        uint64_t send_time_ms = ros::Time::now().toNSec() / 1000000;
        Json j;
        j["master_twist"] = zbase64::encode(serialize_ros(master_twist));
        j["master_odom"] = zbase64::encode(serialize_ros(master_odom_));
        j["send_time_ms"] = send_time_ms;
        j["desired_gap"] = desired_gap_;
        j["formation_type"] = 0;  // 链式编队

        // 发送给从车
        mp_server->Send("client-reliable", "formation_reference", j.dump());
        mp_server->Send("client-reliable1", "formation_reference", j.dump());
        PRINT_INFO("发送编队参考信息 - 时间戳: %llu ms", send_time_ms);
    }

    void logMasterControlStatus(const geometry_msgs::Twist& master_twist)
    {
        static int log_counter = 0;
        if (++log_counter % 10 == 0) {
            PRINT_INFO("Master控制状态 - v=%.2f, ω=%.2f",
                      master_twist.linear.x, master_twist.angular.z);
        }
    }

    /**
     * @brief 记录主车位置信息
     */
    void logMasterPosition()
    {
        static int position_log_counter = 0;
        if (++position_log_counter % 5 == 0 && have_master_odom_) {
            double x = master_odom_.pose.pose.position.x;
            double y = master_odom_.pose.pose.position.y;
            double yaw = tf::getYaw(master_odom_.pose.pose.orientation);

            // 计算实际速度
            double v = std::sqrt(
                master_odom_.twist.twist.linear.x * master_odom_.twist.twist.linear.x +
                master_odom_.twist.twist.linear.y * master_odom_.twist.twist.linear.y
            );
            double omega = master_odom_.twist.twist.angular.z;

            PRINT_INFO("Master实时位置 - x=%.3f, y=%.3f, yaw=%.3f, v=%.3f, ω=%.3f",
                      x, y, yaw, v, omega);
        }
    }

    // ==================== 内部工具函数 ====================
    bool HaveOdometry()
    {
        return have_master_odom_ && have_slave_odom_1 && have_slave_odom_2;
    }

    /**
     * @brief 角度归一化（-π ~ π）
     * @param a 原始角度
     * @return 归一化后的角度
     */
    double NormalizeAngle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

private:
    /**
     * @brief 初始化MPC参数
     */
    void initMPCParams()
    {
        master_mpc_.setParameters(10, 0.1, 5.0, 2.0, 10.0, 0.1, 0.1);
        master_mpc_.setLimits(0.8, 1.5);
    }

    // ==================== 成员变量 ====================
    SimpleFormationNMPCController master_mpc_;  // MPC控制器实例

    // 通信相关
    ZmqServer* mp_server;
    ros::NodeHandle m_nh;
    std::map<std::string, ros::Subscriber> m_subers;  // 订阅器映射
    std::map<std::string, ros::Publisher> m_pubers;    // 发布器映射

    // 指令相关
    std::deque<jarvis_msgs::Cmd> m_send_cmd_queue;
    jarvis_msgs::Cmd m_last_recv_cmd;

    // 车辆参数
    double L_;                // 轴距
    double W_;                // 轮距
    double L_split_ = 2.0;    // 分割长度
    static constexpr double eps = 1e-8;  // 数值精度

    // 编队控制参数
    double desired_gap_ = 1.5;  // 期望间距

    // 里程计数据
    nav_msgs::Odometry master_odom_;
    nav_msgs::Odometry slave_odom_1;
    nav_msgs::Odometry slave_odom_2;
    bool have_master_odom_ = false;
    bool have_slave_odom_1 = false;
    bool have_slave_odom_2 = false;

    // 路径相关
    nav_msgs::Path global_path_;
    bool have_global_path_ = false;
    nav_msgs::Path local_path_;
    bool have_local_path_ = false;

    // 控制定时器
    ros::SteadyTimer m_control_timer;
};
