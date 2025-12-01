#include <iostream>
#include <ros/ros.h>
#include "zbase64.h"
#include "common.h"
#include "simple_log.h"
#include "zmq_server.h"
#include "master_ros_node.h"
#include "ros_utils.h"

#include "nav_msgs/Odometry.h"

ZmqServer ZMQ_SERVER;
MasterRosNode* P_ROS_NODE = nullptr;

// ZMQ 调度
void TimerZmqServer(const ros::SteadyTimerEvent& event) {
    ZMQ_SERVER.Run();
}

// 回调：激光（
std::string HanleLaserData(const Json& j) {
    PRINT_DEBUG("recv laser: %.3f", Clock::time_stamp());
    return "";
}

// 回调：从机电机数据
std::string HandleSlaveMotor(const Json& j) {
    const std::string& data = JsonUtils::get_json_data<std::string>(j, "data", "");
    auto msg = RosUtils::deserialize_ros<jarvis_msgs::MotorAckerman>(data);
    if (P_ROS_NODE) {
        P_ROS_NODE->SetSlaveMotor(msg);
    }
    return "";
}

// 回调：从机 info 数据
std::string HandleSlaveInfo(const Json& j) {
    const std::string& data = JsonUtils::get_json_data<std::string>(j, "data", "");
    auto msg = RosUtils::deserialize_ros<jarvis_msgs::Info>(data);
    if (P_ROS_NODE) {
        P_ROS_NODE->SetSlaveInfo(msg);
    }
    return "";
}

// 回调：从机 odom 数据
std::string HandleSlaveOdom_1(const Json& j) {
    const std::string encoded = JsonUtils::get_json_data<std::string>(j, "data", "");
    const std::string raw = zbase64::decode(encoded);
    nav_msgs::Odometry msg = RosUtils::deserialize_ros<nav_msgs::Odometry>(raw);

    if (P_ROS_NODE) {
        P_ROS_NODE->SetSlaveOdom_1(msg);
    }
    return "";
}
std::string HandleSlaveOdom_2(const Json& j) {
    const std::string encoded = JsonUtils::get_json_data<std::string>(j, "data", "");
    const std::string raw = zbase64::decode(encoded);
    nav_msgs::Odometry msg = RosUtils::deserialize_ros<nav_msgs::Odometry>(raw);

    if (P_ROS_NODE) {
        P_ROS_NODE->SetSlaveOdom_2(msg);
    }
    return "";
}
std::string HandleSlaveLaser_0(const Json& j) {
    const std::string encoded = JsonUtils::get_json_data<std::string>(j, "data", "");
    const std::string raw = zbase64::decode(encoded);
    sensor_msgs::LaserScan msg = RosUtils::deserialize_ros<sensor_msgs::LaserScan>(raw);

    if (P_ROS_NODE) {
        P_ROS_NODE->SetSlaveLaser_0(msg);
    }
    return "";
}
std::string HandleSlaveLaser_1(const Json& j) {
    const std::string encoded = JsonUtils::get_json_data<std::string>(j, "data", "");
    const std::string raw = zbase64::decode(encoded);
    sensor_msgs::LaserScan msg = RosUtils::deserialize_ros<sensor_msgs::LaserScan>(raw);

    if (P_ROS_NODE) {
        P_ROS_NODE->SetSlaveLaser_1(msg);
    }
    return "";
}

int main(int argc, char** argv)
{
    try {
        SimpleLogger& logger = SimpleLogger::getInstance(log_dir, log_name, max_files, max_size);
    } catch (const std::exception& ex) {
        std::cerr << "Logger initialization failed: " << ex.what() << std::endl;
    }
    PRINT_INFO("zmq server start");
    PRINT_INFO("%s", __FILE__);
    PRINT_INFO("compile on: %s - %s", __DATE__, __TIME__);

    // 初始化 ZMQ
    if(argc == 2) {
        ZMQ_SERVER.Init(argv[1]);
    } else {
        ZMQ_SERVER.Init("tcp://*:5000");
        // 同时监听5001端口用于slave1
        ZMQ_SERVER.Init("tcp://*:5001");//目前该端口只能监听，通信send功能暂不支持
    }

    ros::init(argc, argv, "zmq_server");
    ros::NodeHandle nh;

    // 读取机器人参数
    const std::string file_path = "master_node.json";
    Json j = JsonUtils::read_json_from_file(file_path);
    float wheel_base  = JsonUtils::get_json_data<float>(j, "wheel_base", 1.0f);
    float track_width = JsonUtils::get_json_data<float>(j, "track_width", 0.8f);

    // 构建 ROS 节点
    P_ROS_NODE = new MasterRosNode(&ZMQ_SERVER, wheel_base, track_width);

    // 注册 ZMQ 回调
    ZMQ_SERVER.RegisterCmd("slave_motor_akm", HandleSlaveMotor);
    ZMQ_SERVER.RegisterCmd("slave_info", HandleSlaveInfo);
    ZMQ_SERVER.RegisterCmd("Slave_Odom_1", HandleSlaveOdom_1);
    ZMQ_SERVER.RegisterCmd("Slave_Odom_2", HandleSlaveOdom_2);
    ZMQ_SERVER.RegisterCmd("SlaveLaser_0", HandleSlaveLaser_0);
    ZMQ_SERVER.RegisterCmd("SlaveLaser_1", HandleSlaveLaser_1);//测试用

    // 定时调度
    auto timer_zmq_server = nh.createSteadyTimer(ros::WallDuration(0.01), TimerZmqServer);

    ros::spin();
    return 0;
}
