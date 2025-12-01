#include <iostream>
#include <ros/ros.h>
#include "simple_log.h"
#include "zmq_client.h"
#include "zmq_server.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "zbase64.h"
#include "lz77.h"
#include  "common.h"
#include "slave_ros_node.h"

#include <iomanip>
#include <nav_msgs/Path.h>

ZmqClient ZMQ_CLIENT;
SlaveRosNode* P_ROS_NODE = nullptr;
// 添加全局变量存储激光数据
sensor_msgs::LaserScan g_master_laser_data_0;
sensor_msgs::LaserScan g_master_laser_data_1;

std::vector<SimpleFormationNMPCController::RefPoint> ref_slave;

void TimerZmqClient(const ros::SteadyTimerEvent& event) {
    //PRINT_DEBUG("从节点定时器触发，调用ZMQ_CLIENT.Run()");
    ZMQ_CLIENT.Run();
}

std::string HandleMasterLaser0(const Json& j) {
    try {
        //PRINT_DEBUG("Received 主激光0数据的时间 at: %.3f", Clock::time_stamp());

        const std::string& laser_data0 = JsonUtils::get_json_data<std::string>(j, "data", "");
        if (laser_data0.empty()) {
            PRINT_WARN("Empty laser data received in HandleMasterLaser0");
            return "";
        }
        // 接收时间
        ros::Time recv_time = ros::Time::now();
        g_master_laser_data_0 = RosUtils::deserialize_ros<sensor_msgs::LaserScan>(zbase64::decode(laser_data0));
        // 计算延迟 (ms)
        double send_time = g_master_laser_data_0.header.stamp.toSec() * 1000.0;
        double recv_time_ms = recv_time.toSec() * 1000.0;
        double delay_ms = recv_time_ms - send_time;
        PRINT_INFO("接收master laser 所需时间：%.f ms",delay_ms);

    } catch (const std::exception& e) {
        PRINT_ERROR("Error handling master laser data: %s", e.what());
    }

    return "";  // 空回复
}
std::string HandleMasterLaser1(const Json& j) {
    try {
        //PRINT_DEBUG("Received 主激光1数据的时间 at: %.3f", Clock::time_stamp());

        const std::string& laser_data1 = JsonUtils::get_json_data<std::string>(j, "data", "");
        if (laser_data1.empty()) {
            PRINT_WARN("Empty laser data received in HandleMasterLaser0");
            return "";
        }

        g_master_laser_data_1 = RosUtils::deserialize_ros<sensor_msgs::LaserScan>(zbase64::decode(laser_data1));

        // 更新时间戳（避免数据过期警告）
        g_master_laser_data_1.header.stamp = ros::Time::now();
    } catch (const std::exception& e) {
        PRINT_ERROR("Error handling master laser data: %s", e.what());
    }
    return "";  // 空回复
}

std::string HandleFormationReference(const Json& j) {
    try {
        PRINT_DEBUG("Received formation reference at: %.3f", Clock::time_stamp());

        // 解析编队参考信息
        const std::string& master_twist_data = JsonUtils::get_json_data<std::string>(j, "master_twist", "");
        const std::string& master_odom_data = JsonUtils::get_json_data<std::string>(j, "master_odom", "");
        double desired_gap = JsonUtils::get_json_data<double>(j, "desired_gap", 1.5);
        int formation_type = JsonUtils::get_json_data<int>(j, "formation_type", 0);
        uint64_t send_time_ms = JsonUtils::get_json_data<uint64_t>(j, "send_time_ms", 0);

        if (master_twist_data.empty() || master_odom_data.empty()) {
            PRINT_WARN("Empty formation reference data received");
            return "error: empty data";
        }

        // 反序列化数据
        geometry_msgs::Twist master_twist = RosUtils::deserialize_ros<geometry_msgs::Twist>(zbase64::decode(master_twist_data));
        nav_msgs::Odometry master_odom = RosUtils::deserialize_ros<nav_msgs::Odometry>(zbase64::decode(master_odom_data));

        // 计算通信延迟
        uint64_t recv_time_ms = ros::Time::now().toNSec() / 1000000;
        double delay_ms = static_cast<double>(recv_time_ms - send_time_ms);

        PRINT_INFO("收到编队参考信息 - 延迟: %.2fms, 期望间距: %.2fm, 编队类型: %d",
                   delay_ms, desired_gap, formation_type);
        PRINT_INFO("Master速度: v=%.2f, ω=%.2f", master_twist.linear.x, master_twist.angular.z);

        // 将参考信息传递给SlaveRosNode
        if (P_ROS_NODE) {
            P_ROS_NODE->SetFormationReference(master_twist, master_odom, desired_gap, formation_type, send_time_ms);
        } else {
            PRINT_ERROR("P_ROS_NODE 未初始化，无法设置编队参考信息");
            return "error: P_ROS_NODE 为空";
        }

    } catch (const std::exception& e) {
        PRINT_ERROR("Error handling formation reference: %s", e.what());
        return "error: exception occurred";
    }

    return "ok";
}

std::string Receivelocalpath(const Json& j) {
    try {
        const std::string& ref_base64 = JsonUtils::get_json_data<std::string>(j, "data", "");
        if (ref_base64.empty()) {
            PRINT_WARN("Empty formation reference data received");
            return "error: empty data";
        }

        // 步骤1：Base64 解码（与发送端 zbase64::encode 对应）
        std::string ref_serialized = zbase64::decode(ref_base64);
        if (ref_serialized.empty()) {
            PRINT_ERROR("Base64 decode failed for formation reference");
            return "error: base64 decode failed";
        }

        // 步骤2：反序列化（模板参数与发送端完全一致！）
        // 发送端序列化的是 vector<RefPoint>，接收端直接反序列化为该类型
        std::vector<SimpleFormationNMPCController::RefPoint> ref_slave =
            RosUtils::deserialize_ros<std::vector<jarvis_msgs::RefPoint>>(ref_serialized);

        // 步骤3：验证反序列化结果
        if (ref_slave.empty()) {
            PRINT_WARN("Deserialized formation reference is empty");
            return "error: empty ref_slave after deserialization";
        }

        // 步骤4：传递给 SlaveRosNode（与之前逻辑一致）
        if (P_ROS_NODE) {
            P_ROS_NODE->Set_ref(ref_slave);
            PRINT_INFO("Successfully set formation reference: %zu points", ref_slave.size());
            // 打印第一个点信息，确认数据正确
            const auto& first = ref_slave[0];
            PRINT_DEBUG("First ref point: x=%.3f, y=%.3f, yaw=%.3f, v=%.3f",
                       first.x, first.y, first.yaw, first.v);
        } else {
            PRINT_ERROR("P_ROS_NODE 未初始化，无法设置编队参考信息");
            return "error: P_ROS_NODE 为空";
        }

    } catch (const std::exception& e) {
        PRINT_ERROR("Error handling formation reference: %s", e.what());
        return "error: exception occurred";
    }

    return "ok";
}

int main(int argc, char** argv)
{
    try {
        SimpleLogger& logger = SimpleLogger::getInstance(log_dir, log_name, max_files, max_size);
    } catch (const std::exception& ex) {
        std::cerr << "Logger initialization failed: " << ex.what() << std::endl;
    }
    PRINT_INFO("zmq client start");
    PRINT_INFO("%s", __FILE__);
    PRINT_INFO("compile on: %s - %s", __DATE__, __TIME__);

    PRINT_INFO("************* client 注册 *************");
    ros::init(argc, argv, "zmq_client", ros::InitOption::NoSigintHandler);

    ros::NodeHandle nh;
    if(argc == 2)
    {
        printf("zmq client connect addr = %s\n", argv[1]);
        PRINT_INFO("zmq client connect addr = %s\n", argv[1]);
        ZMQ_CLIENT.Init(argv[1], "client-reliable");
    }
    else {
        PRINT_INFO("zmq client connect addr = %s\n", "tcp://127.0.0.1:5000");
        ZMQ_CLIENT.Init("tcp://127.0.0.1:5000", "client-reliable");
    }

    // 注册 ZMQ 命令回调
//    ZMQ_CLIENT.RegisterCmd("cmd_vel_from_master", HandleMasterCmdVel);
    ZMQ_CLIENT.RegisterCmd("masterLaser0", HandleMasterLaser0);
    ZMQ_CLIENT.RegisterCmd("masterLaser1", HandleMasterLaser1);
    ZMQ_CLIENT.RegisterCmd("formation_reference", HandleFormationReference);//获取master的里程计和速度
    ZMQ_CLIENT.RegisterCmd("local_path_ref", Receivelocalpath);//获取参考轨迹
    // 创建 ROS 层封装节点（包含队列与发布逻辑）
    P_ROS_NODE = new SlaveRosNode(&ZMQ_CLIENT);
    auto timer_zmq_client = nh.createSteadyTimer(ros::WallDuration(0.01), TimerZmqClient);


    // auto laser_suber1 = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, boost::bind( RosSetLaser, _1, "/base_scan")) ;
    // auto laser_suber2 = nh.subscribe<sensor_msgs::LaserScan>("/base_scan_0", 1, boost::bind( RosSetLaser, _1, "/base_scan_0")) ;
    ros::spin();
    return 0;
}
