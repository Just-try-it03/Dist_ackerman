//
// Created by BlindZhou on  2025-06-17.
//

#include <ros/ros.h>

#pragma once

class RosUtils {
public:
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
        ros::serialization::IStream stream(((uint8_t*)buffer.data()), buffer.size());
        ros::serialization::deserialize(stream, msg_ros);
        return msg_ros;
    }

};