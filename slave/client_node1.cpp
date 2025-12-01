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
#include "slave_ros_node1.h"

#include <iomanip>

ZmqClient ZMQ_CLIENT1;
SlaveRosNode1* P_ROS_NODE1 = nullptr;

void TimerZmqClient1(const ros::SteadyTimerEvent& event) {
    //PRINT_DEBUG("从节点1定时器触发，调用ZMQ_CLIENT1.Run()");
    ZMQ_CLIENT1.Run();
}

std::string HandleFormationReference1(const Json& j) {
    try {
        PRINT_DEBUG("Received formation reference at: %.3f", Clock::time_stamp());

        // 解析编队参考信息
        const std::string& master_twist_data = JsonUtils::get_json_data<std::string>(j, "master_twist", "");
        const std::string& master_odom_data = JsonUtils::get_json_data<std::string>(j, "master_odom", "");
        double desired_gap = JsonUtils::get_json_data<double>(j, "desired_gap", 1.5)+1.0;// 需要修改此处逻辑
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
        if (P_ROS_NODE1) {
            P_ROS_NODE1->SetFormationReference1(master_twist, master_odom, desired_gap, formation_type, send_time_ms);
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
std::string Receivelocalpath1(const Json& j) {
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

        // 步骤2：反序列化（模板参数与发送端完全一致！关键修正）
        // 发送端序列化的是 vector<RefPoint>，接收端直接反序列化为该类型
        std::vector<SimpleFormationNMPCController::RefPoint> ref_slave =
            RosUtils::deserialize_ros<std::vector<jarvis_msgs::RefPoint>>(ref_serialized);

        // 步骤3：验证反序列化结果
        if (ref_slave.empty()) {
            PRINT_WARN("Deserialized formation reference is empty");
            return "error: empty ref_slave after deserialization";
        }

        // 步骤4：传递给 SlaveRosNode（与之前逻辑一致）
        if (P_ROS_NODE1) {
            P_ROS_NODE1->Set_ref(ref_slave);
            PRINT_INFO("Successfully set formation reference: %zu points", ref_slave.size());
            // 可选：打印第一个点信息，确认数据正确
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

int main(int argc, char** argv)  //（命令行参数的数量 命令行参数的字符串数组）
{
    try {
        SimpleLogger& logger = SimpleLogger::getInstance(log_dir, log_name, max_files, max_size); //第一步 初始化日志系统
    } catch (const std::exception& ex) {
        std::cerr << "Logger initialization failed: " << ex.what() << std::endl;
    }
    PRINT_INFO("zmq client1 start");
    PRINT_INFO("%s", __FILE__);
    PRINT_INFO("compile on: %s - %s", __DATE__, __TIME__);

    PRINT_INFO("************* client1 注册 *************");
    ros::init(argc, argv, "zmq_client1", ros::InitOption::NoSigintHandler);// 第二步 初始化ROS节点

    ros::NodeHandle nh;
    if(argc == 2)   // 第三步 初始化ZMQ客户端
    {               //使用命令行参数
        printf("zmq client1 connect addr = %s\n", argv[1]);
        PRINT_INFO("zmq client1 connect addr = %s\n", argv[1]);
        //ZMQ_CLIENT1.Init(argv[1]);
        ZMQ_CLIENT1.Init(argv[1], "client-reliable1");
    }
    else {         //使用默认参数
        PRINT_INFO("zmq client1 connect addr = %s\n", "tcp://127.0.0.1:5000");
        //ZMQ_CLIENT1.Init("tcp://127.0.0.1:5001");
        ZMQ_CLIENT1.Init("tcp://127.0.0.1:5000", "client-reliable1");
    }

    ZMQ_CLIENT1.RegisterCmd("formation_reference", HandleFormationReference1);//获取master的里程计和速度
    ZMQ_CLIENT1.RegisterCmd("local_path_ref", Receivelocalpath1);//获取参考轨迹

    // 第四步 注册 ZMQ 命令回调
    //ZMQ_CLIENT1.RegisterCmd("set_slave_vel", SetSlaveVel1);

    P_ROS_NODE1 = new SlaveRosNode1(&ZMQ_CLIENT1);  // 第五步 创建ROS封装节点
    auto timer_zmq_client1 = nh.createSteadyTimer(ros::WallDuration(0.01), TimerZmqClient1); // 第六步 创建定时器


    // auto laser_suber1 = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, boost::bind( RosSetLaser, _1, "/base_scan")) ;
    // auto laser_suber2 = nh.subscribe<sensor_msgs/LaserScan>("/base_scan_0", 1, boost::bind( RosSetLaser, _1, "/base_scan_0")) ;
    ros::spin();  // 第七步 进入主循环
    return 0;
}
