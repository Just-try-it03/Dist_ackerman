//
// Created by ucore on 5/14/25.
//
#ifndef ZMQ_UTIL_H
#define ZMQ_UTIL_H
#include <stdint.h>
#include <string>
#include <chrono>

//#define  zmq::send_flags::none 0
#define  ZMQ_SEND_FLAGS_NONE 0

struct PendingMessage {
    std::string msg_id;
    std::string content;
    //std::chrono::steady_clock::time_point timestamp;
    int64_t time_stamp_us;
    int retry_count = 0;
    uint32_t msg_seq_0;
    uint32_t msg_seq_1;
};

std::string GenMsgId();

uint64_t GenMsgSeq();

std::string GenUuid();

#endif //ZMQ_UTIL_H
