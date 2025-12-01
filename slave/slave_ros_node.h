#pragma once

// 系统标准库头文件
#include <iostream>
#include <map>
#include <deque>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include "jarvis_msgs/Odom.h"
#include "jarvis_msgs/Info.h"
#include "jarvis_msgs/RobotStatus.h"
#include "jarvis_msgs/LocResult.h"
#include "jarvis_msgs/Cmd.h"
#include "jarvis_msgs/Json.h"
#include "jarvis_msgs/Clearance.h"
#include "jarvis_msgs/VelSteer.h"
#include "jarvis_msgs/MotorAckerman.h"
#include "jarvis_msgs/VelAckerman.h"

// 自定义工具头文件
#include "common.h"
#include "zmq_client.h"
#include "ros_utils.h"
#include "mpc_controller_nmpc.h"

class SlaveRosNode {
public:
    SlaveRosNode(ZmqClient* p_client) : mp_client(p_client)
    {
        m_last_recv_cmd.cmd = -1;
        SlaveInit();

        // 订阅器初始化
        m_subers["odom"] = m_nh.subscribe<jarvis_msgs::Odom>(
            "/jodom_ackermann", 1, &SlaveRosNode::SetOdom, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["motor_slave"] = m_nh.subscribe<jarvis_msgs::MotorAckerman>(
            "/jmotor_ackermann", 1, &SlaveRosNode::SetMotorReal, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["jinfo"] = m_nh.subscribe<jarvis_msgs::Info>(
            "/jinfo", 1, &SlaveRosNode::SetRobotInfo, this);
        m_subers["cmd_res"] = m_nh.subscribe(
            "/jcmd_res", 1000, &SlaveRosNode::SetCmdRes, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["laser"] = m_nh.subscribe(
            "/front_scan_filtered", 1, &SlaveRosNode::SetLaserScan, this);

        m_subers["slave_odom_1"] = m_nh.subscribe<nav_msgs::Odometry>(
            "/robot_1/odom", 1, &SlaveRosNode::SlaveOdom_stage, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["slave1_laser_0"] = m_nh.subscribe<sensor_msgs::LaserScan>(
            "/robot_1/base_scan_0", 1, &SlaveRosNode::SlaveLaser0Callback, this,
            ros::TransportHints().tcpNoDelay());
        m_subers["slave1_laser_1"] = m_nh.subscribe<sensor_msgs::LaserScan>(
            "/robot_1/base_scan_1", 1, &SlaveRosNode::SlaveLaser1Callback, this,
            ros::TransportHints().tcpNoDelay());

        // 发布器初始化
        m_pubers["cmd"] = m_nh.advertise<jarvis_msgs::Cmd>("/jcmd", 1);
        m_pubers["cmd_vel_1"] = m_nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1);

        // 控制定时器：100ms周期（10Hz）
        m_control_timer = m_nh.createSteadyTimer(
            ros::WallDuration(0.1), &SlaveRosNode::CycleControlCallbackSlave, this);
    }

    /**
     * @brief 析构函数
     */
    ~SlaveRosNode() = default;

    // ==================== 外部接口 ====================
    void SetFormationReference(const geometry_msgs::Twist& master_twist,
                             const nav_msgs::Odometry& master_odom,
                             double desired_gap,
                             int formation_type,
                             uint64_t send_time_ms) {
        std::lock_guard<std::mutex> lock(formation_ref_mutex_);

        // 更新编队参考信息
        formation_ref_.master_twist = master_twist;
        formation_ref_.master_odom = master_odom;
        formation_ref_.desired_gap = desired_gap;
        formation_ref_.formation_type = formation_type;
        formation_ref_.send_time_ms = send_time_ms;
        formation_ref_.received_time_ms = ros::Time::now().toNSec() / 1000000;

        // 计算通信延迟
        double delay_ms = static_cast<double>(formation_ref_.received_time_ms - send_time_ms);

        PRINT_INFO("设置编队参考信息 - 延迟: %.2fms, 期望间距: %.2fm", delay_ms, desired_gap);
        have_formation_ref_ = true;
    }

    void Set_ref(const std::vector<SimpleFormationNMPCController::RefPoint>& ref_slave) {
        ref_trajectory_ = ref_slave;
        slave_mpc_.setReferencePath(ref_trajectory_);
    }

    void CycleControlCallbackSlave(const ros::SteadyTimerEvent& event) {
        // 检查是否有odom数据
        if (!have_slave_odom_) {
            PRINT_WARN("Slave odom数据不可用，跳过控制周期");
            return;
        }

        FormationReference ref = receiveFormationReference();

        geometry_msgs::Twist slave_twist = SlaveCycleControlCallback(ref);

        PublishTwist1(slave_twist);

        logSlaveControlStatus(slave_twist);
    }

    void PublishTwist1(const geometry_msgs::Twist& slave_twist_1) {
        auto it = m_pubers.find("cmd_vel_1");
        if (it != m_pubers.end() && it->second) {
            it->second.publish(slave_twist_1);
        }
    }

    /**
     * @brief 应用阿克曼速度指令
     * @param msg 阿克曼速度消息
     */
    void ApplyVelAckerman(const jarvis_msgs::VelAckerman& msg) {
        PRINT_INFO("Apply to slave: v_fleft = %.2f, v_fright = %.2f, angle = %.2f",
                   msg.v_fleft, msg.v_fright, msg.angle_front);
    }

    /**
     * @brief 发布控制指令
     * @param cmd 指令类型
     * @param arg8 8位参数
     * @param arg32 32位参数
     * @param str 字符串参数
     */
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

private:
    // ==================== 内部回调函数 ====================
    /**
     * @brief 里程计回调（阿克曼里程计）
     * @param odom 里程计消息指针
     */
    void SetOdom(const jarvis_msgs::Odom::ConstPtr& odom) {
        if (mp_client) {
            mp_client->Send("slave_odom", RosUtils::serialize_ros(*odom));
        }
    }

    /**
     * @brief 从车1里程计回调
     * @param msg 里程计消息指针
     */
    void SlaveOdom_stage(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double yaw = tf::getYaw(msg->pose.pose.orientation);

        // 数据有效性校验
        if (std::isnan(x) || std::isnan(y) || std::isnan(yaw) ||
            fabs(x) > 1000 || fabs(y) > 1000) {
            PRINT_WARN("Slave odom数据异常，跳过更新！x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
            return;
        }

        slave_odom_ = *msg;
        have_slave_odom_ = true;
    }

    /**
     * @brief 从车激光0数据回调
     * @param laser 激光扫描消息指针
     */
    void SlaveLaser0Callback(const sensor_msgs::LaserScan::ConstPtr& laser)
    {
        try {
            if (!laser || laser->ranges.empty()) {
                PRINT_WARN("Received empty laser data");
                return;
            }

            Json j0;
            j0["data"] = zbase64::encode(RosUtils::serialize_ros(*laser));
            mp_client->Send("SlaveLaser_0", j0.dump());
        } catch (const std::exception& e) {
            PRINT_ERROR("Error processing slave laser0 data: %s", e.what());
        }
    }

    /**
     * @brief 从车激光1数据回调
     * @param laser 激光扫描消息指针
     */
    void SlaveLaser1Callback(const sensor_msgs::LaserScan::ConstPtr& laser)
    {
        try {
            if (!laser || laser->ranges.empty()) {
                PRINT_WARN("Received empty laser data");
                return;
            }

            Json j1;
            j1["data"] = zbase64::encode(RosUtils::serialize_ros(*laser));
            mp_client->Send("SlaveLaser_1", j1.dump());
        } catch (const std::exception& e) {
            PRINT_ERROR("Error processing slave laser1 data: %s", e.what());
        }
    }

    /**
     * @brief 激光扫描数据回调
     * @param laser 激光扫描消息
     */
    void SetLaserScan(const sensor_msgs::LaserScan& laser) {
        mp_client->Send("slave_laser", RosUtils::serialize_ros(laser));
    }

    /**
     * @brief 电机数据回调（从车）
     * @param msg 电机阿克曼消息指针
     */
    void SetMotorReal(const jarvis_msgs::MotorAckerman::ConstPtr& msg)
    {
        mp_client->Send("slave_motor_akm", RosUtils::serialize_ros(*msg));
    }

    /**
     * @brief 机器人信息回调
     * @param msg 机器人信息消息指针
     */
    void SetRobotInfo(const jarvis_msgs::Info::ConstPtr& msg)
    {
        mp_client->Send("slave_info", RosUtils::serialize_ros(*msg));
        CheckCmd();
    }

    /**
     * @brief 设置机器人状态
     * @param msg 机器人状态消息
     */
    void SetRobotStatus(const jarvis_msgs::RobotStatus& msg)
    {
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
                PRINT_INFO("publish cmd: [%d %d %d %s] success",
                           sent_cmd.cmd, sent_cmd.data.arg_int8,
                           sent_cmd.data.arg_int32, sent_cmd.data.arg_str.c_str());
            } else {
                m_pubers["cmd"].publish(sent_cmd);
                PRINT_INFO("retry publish cmd: [%d %d %d %s]",
                           sent_cmd.cmd, sent_cmd.data.arg_int8,
                           sent_cmd.data.arg_int32, sent_cmd.data.arg_str.c_str());
            }
        }
    }

    // ==================== 编队控制相关 ====================
    /**
     * @brief 编队参考信息结构体（扩展版本）
     */
    struct FormationReference {
        geometry_msgs::Twist master_twist;
        nav_msgs::Odometry master_odom;
        double desired_gap;
        int formation_type;
        uint64_t send_time_ms;
        uint64_t received_time_ms;
    };

    FormationReference receiveFormationReference() {
        FormationReference ref;
        std::lock_guard<std::mutex> lock(formation_ref_mutex_);

        if (have_formation_ref_) {
            // 使用实际接收到的编队参考信息
            ref = formation_ref_;

            // 检查数据新鲜度（超过500ms认为数据过期）
            uint64_t current_time_ms = ros::Time::now().toNSec() / 1000000;
            double data_age_ms = static_cast<double>(current_time_ms - ref.received_time_ms);

            if (data_age_ms > 500.0) {
                PRINT_WARN("编队参考信息已过期: %.2fms，使用默认值", data_age_ms);
                // 数据过期，返回默认值
                ref.master_twist.linear.x = 0.0;
                ref.master_twist.angular.z = 0.0;
                ref.desired_gap = 1.5;
                ref.formation_type = 0;
            } else {
                PRINT_DEBUG("使用实际编队参考信息 - 数据年龄: %.2fms", data_age_ms);
            }
        } else {
            // 没有接收到编队参考信息，使用默认值
            PRINT_WARN("未接收到编队参考信息，使用默认值");
            ref.master_twist.linear.x = 0.0;
            ref.master_twist.angular.z = 0.0;
            ref.desired_gap = 1.5;
            ref.formation_type = 0;
            ref.send_time_ms = ros::Time::now().toNSec() / 1000000;
        }

        return ref;
    }
    /**
     * @brief 从车周期控制计算（MPC版本）
     * @param ref 编队参考信息
     * @return 从车速度指令
     */
    geometry_msgs::Twist SlaveCycleControlCallback(const FormationReference& ref) {
        geometry_msgs::Twist out;

        if (!have_slave_odom_) {
            PRINT_WARN("Slave odom数据不可用，返回零速度");
            return out;
        }

        double slave_x = slave_odom_.pose.pose.position.x;
        double slave_y = slave_odom_.pose.pose.position.y;
        double slave_yaw = tf::getYaw(slave_odom_.pose.pose.orientation);
        double slave_v = std::sqrt(
            slave_odom_.twist.twist.linear.x * slave_odom_.twist.twist.linear.x +
            slave_odom_.twist.twist.linear.y * slave_odom_.twist.twist.linear.y
        );

        double master_x = ref.master_odom.pose.pose.position.x;
        double master_y = ref.master_odom.pose.pose.position.y;
        double master_yaw = tf::getYaw(ref.master_odom.pose.pose.orientation);
        double master_v = ref.master_twist.linear.x;
        double master_omega = ref.master_twist.angular.z;

        PRINT_INFO("master和slave日志测试 %.3f %.3f %.3f %.3f %.3f",
                   slave_x, slave_y, slave_yaw, master_x, master_v);

        const int N = 10;  // 要和 controller 里 horizon 一致
        const double dt = 0.1;

        std::vector<SimpleFormationNMPCController::LeaderPoint> leader_traj;
        leader_traj.reserve(N + 1);

        double x = master_x;
        double y = master_y;
        double yaw = master_yaw;

        for (int k = 0; k <= N; ++k) {
            SimpleFormationNMPCController::LeaderPoint p;
            p.x = x;
            p.y = y;
            p.yaw = yaw;
            p.v = master_v;
            p.omega = master_omega;
            leader_traj.push_back(p);

            // 用当前 v, omega 做简单前向积分
            x   += master_v * std::cos(yaw) * dt;
            y   += master_v * std::sin(yaw) * dt;
            yaw += master_omega * dt;
            yaw  = SimpleFormationNMPCController::normalizeAngle(yaw);
        }

        PRINT_INFO("leader_traj 的轨迹点数量：%zu", leader_traj.size());
        slave_mpc_.setLeaderTrajectory(leader_traj);

        // slave 当前位姿
        std::vector<double> pose(3);
        pose[0] = slave_x;
        pose[1] = slave_y;
        pose[2] = slave_yaw;

        // 计算 slave 控制量
        std::vector<double> cmd = slave_mpc_.computeControl(pose);
        out.linear.x  = cmd[0];
        out.angular.z = cmd[1];

        return out;
    }

    /**
     * @brief 应用slave安全约束
     * @param twist 原始速度指令
     * @return 约束后的速度指令
     */
    geometry_msgs::Twist applySlaveSafetyConstraints(const geometry_msgs::Twist& twist) {
        geometry_msgs::Twist constrained = twist;

        // 速度约束
        double max_vel = 2.0;
        double max_omega = 1.0;

        constrained.linear.x = std::max(-max_vel, std::min(constrained.linear.x, max_vel));
        constrained.angular.z = std::max(-max_omega, std::min(constrained.angular.z, max_omega));

        return constrained;
    }

    /**
     * @brief 记录slave控制状态
     * @param twist 速度指令
     */
    void logSlaveControlStatus(const geometry_msgs::Twist& twist) {
        static int log_counter = 0;
        log_counter++;

        // 每10次记录一次状态
        if (log_counter % 10 == 0) {
            PRINT_INFO("Slave控制状态 - v=%.2f, ω=%.2f", twist.linear.x, twist.angular.z);
        }
    }

    /**
     * @brief 从车初始化（MPC参数配置）
     */
    void SlaveInit()
    {
        // 设置MPC核心参数
        slave_mpc_.setParameters(10,
                                 0.1,
                                 3.0,
                                 1.5,
                                 8.0,
                                 0.1,
                                 0.1);
        slave_mpc_.setLimits(0.8, 2.5);

        // 设置编队配置
        SimpleFormationNMPCController::FormationConfig cfg;
        cfg.dx_des = -2.0;
        cfg.dy_des =  0.0;
        cfg.dyaw_des = 0.0;
        cfg.q_pos  = 10.0;
        cfg.q_yaw  = 5.0;
        slave_mpc_.setFormationConfig(cfg);
    }

    // ==================== 成员变量 ====================
    ZmqClient* mp_client;
    ros::NodeHandle m_nh;
    std::map<std::string, ros::Subscriber> m_subers;
    std::map<std::string, ros::Publisher> m_pubers;

    std::deque<jarvis_msgs::Cmd> m_send_cmd_queue;
    jarvis_msgs::Cmd m_last_recv_cmd;

    ros::SteadyTimer m_control_timer;

    // 从车里程计相关
    nav_msgs::Odometry slave_odom_;
    bool have_slave_odom_ = false;

    // 编队控制相关
    FormationReference formation_ref_;
    bool have_formation_ref_ = false;
    std::mutex formation_ref_mutex_;

    SimpleFormationNMPCController slave_mpc_;
    std::vector<SimpleFormationNMPCController::RefPoint> ref_trajectory_;
};
