//
// Created by ucore on 5/14/25.
//

#include "zmq_server.h"
#include "json.h"
#include <random>
#include <iostream>
#include "zbase64.h"
#include "lz77.h"
#include "simple_log.h"
#include "zmq_util.h"
#include "common.h"


ZmqServer::ZmqServer():
    m_context(1)
 {
}

void ZmqServer::Init(const std::string &addr) {
    auto router = std::make_shared<zmq::socket_t>(m_context, zmq::socket_type::router);
    router->bind(addr);
    m_routers.push_back(router);
    PRINT_INFO("ZMQ Server listening on: %s", addr.c_str());
    m_pending_map.clear();
}

void ZmqServer::Run() {
    Recv();
    ReSend();
}

void ZmqServer::Recv() {
    while(true)
    {
        if(!RecvOneMsg())
        {
            break;
        }
    }
}
 bool ZmqServer::RecvOneMsg()
 {
    // 为所有router创建poll items
    std::vector<zmq::pollitem_t> items;
    for (auto& router : m_routers) {
        items.push_back({static_cast<void *>(*router), 0, ZMQ_POLLIN, 0});
    }

    zmq::poll(items.data(), items.size(), 0);

    for (size_t i = 0; i < items.size(); ++i) {
        if (items[i].revents & ZMQ_POLLIN) {
            auto& router = m_routers[i];
            zmq::message_t identity;
            zmq::message_t payload;
            router->recv(&identity);
            router->recv(&payload);

            const std::string sclient_id(static_cast<char*>(identity.data()), identity.size());

            PRINT_INFO("收到来自客户端 %s 的消息，通过端口 %zu", sclient_id.c_str(), i);

            if (payload.size() == 0) {
                return true;
            }

            const std::string str_payload(static_cast<char*>(payload.data()), payload.size());
            auto j = JsonUtils::read_json_from_str(str_payload);
            const std::string type = JsonUtils::get_json_data<std::string>(j, "type", "");
            if (type == "ack") {
                const std::string ack_id = JsonUtils::get_json_data<std::string>(j, "msg_id", "");
                if (m_pending_map.find(sclient_id) == m_pending_map.end()) {
                    m_pending_map[sclient_id] = std::deque<PendingMessage>();
                }
                auto &pending_queue = m_pending_map[sclient_id];

                if (!pending_queue.empty() && pending_queue.front().msg_id == ack_id) {
                    pending_queue.pop_front();
                    ReSend();
                }
            } else if (type == "data") {
                // Send message
                zmq::message_t s_empty(0);
                std::string msg_id = JsonUtils::get_json_data<std::string>(j, "msg_id", "");
                if (!msg_id.empty()) {
                    const Json j_reply =
                    {
                        {"type", "ack"},
                        {"msg_id", msg_id}
                    };
                    router->send(identity,ZMQ_SNDMORE);
                    router->send(s_empty,ZMQ_SNDMORE);

                    zmq::message_t zmsg(j_reply.dump().data(), j_reply.dump().size());
                    router->send(zmsg, ZMQ_SEND_FLAGS_NONE);
                }

                auto it = m_client_msg_seq.find(sclient_id);
                if (it == m_client_msg_seq.end()) {
                    m_client_msg_seq[sclient_id] = 0;
                }
                uint32_t msg_seq_0 = JsonUtils::get_json_data<uint32_t>(j, "msg_seq_0", 0);
                uint32_t msg_seq_1 = JsonUtils::get_json_data<uint32_t>(j, "msg_seq_1", 0);
                uint64_t msg_seq = ((uint64_t) msg_seq_1 << 32) | (uint64_t) msg_seq_0;
                if (msg_seq != 0 && msg_seq == m_client_msg_seq[sclient_id]) {
                    PRINT_INFO("recv msg seq(%lu) == last recv msg_seq(%lu), drop", msg_seq, m_client_msg_seq[sclient_id]);
                    return true;;
                }
                m_client_msg_seq[sclient_id] = msg_seq;

                std::string cmd = JsonUtils::get_json_data<std::string>(j, "cmd", "");
                auto handler_it = m_map_handler.find(cmd);
                if (handler_it != m_map_handler.end()) {
                    const std::string msg_data = JsonUtils::get_json_data<std::string>(j, "data", "");
                    if (msg_data.empty()) {
                        ReliableSend(sclient_id, cmd, "");
                        return true;
                    }
                    Json jdata = JsonUtils::read_json_from_str(msg_data);
                    const auto reply_msg = handler_it->second(jdata);
                    if (!reply_msg.empty()) {
                        ReliableSend(sclient_id, cmd, reply_msg);
                    } else {
                        ReliableSend(sclient_id, cmd, "");
                    }
                }
            }
            return true; // 处理了一个消息
        }
    }
    return false;
 }

void ZmqServer::Send(const std::string client_id, const std::string &cmd, const std::string &msg) {
    std::string msg_id = GenMsgId();
    uint64_t msg_seq = GenMsgSeq();
    uint32_t msg_seq_0 = uint32_t(msg_seq & 0x00000000ffffffff);
    uint32_t msg_seq_1 = uint32_t(msg_seq >> 32);

    //std::string msg1 = zbase64::encode(msg);
    //PRINT_INFO("Base64编码后的数据: %s", msg1.c_str());
    const Json j =
    {
        {"type", "data"},
        {"timestamp", Clock::date()},
        {"msg_id", msg_id},
        {"msg_seq_0", msg_seq_0},
        {"msg_seq_1", msg_seq_1},
        {"cmd", cmd},
        {"data", msg}
    };
    PRINT_INFO("发送消息到客户端 %s，命令: %s", client_id.c_str(), cmd.c_str());
    Send(client_id, j.dump());
    //PRINT_INFO(" 发送的msg: %s",msg.c_str());
   // PRINT_INFO("发送的JSON: %s", j.dump().c_str());
}

void ZmqServer::ReliableSend(const std::string client_id, const std::string &cmd, const std::string &msg) { //参考上面send
    std::string msg_id = GenMsgId();
    uint64_t msg_seq = GenMsgSeq();
    uint32_t msg_seq_0 = uint32_t(msg_seq & 0x00000000ffffffff);
    uint32_t msg_seq_1 = uint32_t(msg_seq >> 32);
    const Json j =
    {
        {"type", "data"},
        {"timestamp", Clock::date()},
        {"msg_id", msg_id},
        {"msg_seq_0", msg_seq_0},
        {"msg_seq_1", msg_seq_1},
        {"cmd", cmd},
        //{"data", base64::encode(lz77::compress(msg))}
        {"data", msg}
    };

    if (m_pending_map.find(client_id) == m_pending_map.end()) {
        m_pending_map[client_id] = std::deque<PendingMessage>();
    }
    auto &pending_queue = m_pending_map[client_id];
    if(pending_queue.size() > 10000)
    {
        PRINT_ERROR("(%s)pending size(%lu) > 10000, clear",client_id.c_str(), pending_queue.size());
        pending_queue.clear();
    }

    pending_queue.push_back({msg_id, j.dump(), 0});

    ReSend(client_id);
}


void ZmqServer::Send(const std::string client_id, const std::string &msg) {
    ReSend(client_id);
    zmq::message_t s_empty(0);
    zmq::message_t zmsg_client_id(client_id.data(), client_id.size());
    zmq::message_t zmsg(msg.data(), msg.size());

    // 发送到所有router
    for (auto& router : m_routers) {
        router->send(zmsg_client_id, ZMQ_SNDMORE | ZMQ_DONTWAIT);
        router->send(s_empty, ZMQ_SNDMORE | ZMQ_DONTWAIT);
        router->send(zmsg, ZMQ_SEND_FLAGS_NONE | ZMQ_DONTWAIT);
    }
    //PRINT_DEBUG("send to(%s): %s", client_id.c_str(), msg.c_str());
}

void ZmqServer::RegisterCmd(const std::string &cmd, std::function<std::string(const Json &j)> func) {
    m_map_handler[cmd] = func;
}

void ZmqServer::ReSend(const std::string client_id) {
    if (m_pending_map.find(client_id) == m_pending_map.end()) {
        return;
    }
    auto &pending_queue = m_pending_map[client_id];

    if (!pending_queue.empty()) {
        PendingMessage &pmsg = pending_queue.front();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = Clock::to_now_ms(pmsg.time_stamp_us);
        if (elapsed < 80 && pmsg.time_stamp_us != 0) {
            return;
        }
        pmsg.time_stamp_us = Clock::time_stamp_us();
        zmq::message_t s_empty(0);
        zmq::message_t zmsg_client_id(client_id.data(), client_id.size());
        zmq::message_t zmsg(pmsg.content.data(), pmsg.content.size());

        // 发送到所有router
        for (auto& router : m_routers) {
            router->send(zmsg_client_id, ZMQ_SNDMORE | ZMQ_DONTWAIT);
            router->send(s_empty, ZMQ_SNDMORE | ZMQ_DONTWAIT);
            router->send(zmsg, ZMQ_SEND_FLAGS_NONE | ZMQ_DONTWAIT);
        }
        //PRINT_DEBUG("Realiable send to(%s): %s", client_id.c_str(), pmsg.content.c_str());
    }
}

void ZmqServer::ReSend() {
    for (auto it = m_pending_map.begin(); it != m_pending_map.end(); ++it) {
        ReSend(it->first);
    }
}
