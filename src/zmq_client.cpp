#include "zmq_client.h"
#include "json.h"
#include <random>
#include <iostream>
#include "zbase64.h"
#include "lz77.h"
#include "simple_log.h"
#include "zmq_util.h"
#include "common.h"

ZmqClient::ZmqClient():
m_context(1),
m_dealer(m_context, zmq::socket_type::dealer)
 {
}

/* 单路由 */
//void ZmqClient::Init(const std::string &addr)
//{
//    m_last_recv_msg_seq = 0;
////    PRINT_DEBUG("create new context");
////    m_context = zmq::context_t(1);
////    PRINT_DEBUG("create new dealer");
////    m_dealer = zmq::socket_t(m_context, zmq::socket_type::dealer);
//    //mp_dealer = std::make_shared<zmq::socket_t>(m_context, zmq::socket_type::dealer);
//    //m_dealer.set(zmq::sockopt::routing_id, "client-reliable");
//    PRINT_DEBUG("set client id");
//    m_dealer.setsockopt(ZMQ_ROUTING_ID, "client-reliable", strlen("client-reliable"));

//    //m_dealer.set(zmq::sockopt::linger, 0);
//    int linger_value = 0;
//    PRINT_DEBUG("set ZMQ_LINGER");
//    m_dealer.setsockopt(ZMQ_LINGER, &linger_value, sizeof(linger_value));
//    PRINT_DEBUG("connect");
//    m_dealer.connect(addr);
//    m_pending.clear();

//}
/* 动态路由 */
void ZmqClient::Init(const std::string &addr, const std::string &client_id)
{
    m_last_recv_msg_seq = 0;
//    PRINT_DEBUG("create new context");
//    m_context = zmq::context_t(1);
//    PRINT_DEBUG("create new dealer");
//    m_dealer = zmq::socket_t(m_context, zmq::socket_type::dealer);
    //mp_dealer = std::make_shared<zmq::socket_t>(m_context, zmq::socket_type::dealer);
    //m_dealer.set(zmq::sockopt::routing_id, "client-reliable");
    PRINT_DEBUG("set client id: %s", client_id.c_str());
    m_dealer.setsockopt(ZMQ_ROUTING_ID, client_id.c_str(), client_id.length());

    //m_dealer.set(zmq::sockopt::linger, 0);
    int linger_value = 0;
    PRINT_DEBUG("set ZMQ_LINGER");
    m_dealer.setsockopt(ZMQ_LINGER, &linger_value, sizeof(linger_value));
    PRINT_DEBUG("connect");
    m_dealer.connect(addr);
    m_pending.clear();

}

void ZmqClient::Run() {
    Recv();
    ReSend();
}

void ZmqClient::Recv() {
    while(true)
    {
        if(!RecvOneMsg())
        {
            break;
        }
    }
}

bool ZmqClient::RecvOneMsg()
{
    zmq::pollitem_t items[] = {
        {static_cast<void *>(m_dealer), 0, ZMQ_POLLIN, 0}
    };
    zmq::poll(items, 1, 0);

    if (items[0].revents & ZMQ_POLLIN) {
        zmq::message_t msg;
        //m_dealer.recv(msg);
        m_dealer.recv(&msg);

        if (msg.size() == 0) {
            return true;
        }
    std::string str(static_cast<char*>(msg.data()), msg.size());
        auto j = JsonUtils::read_json_from_str(str);
        const std::string type = JsonUtils::get_json_data<std::string>(j, "type", "");

        if (type == "ack") {
            const std::string ack_id = JsonUtils::get_json_data<std::string>(j, "msg_id", "");
            if (!m_pending.empty() && m_pending.front().msg_id == ack_id) {
                m_pending.pop_front();
                //std::cout << "Received ack for msg_id: " << ack_id << "\n";
                //PRINT_INFO("Received ack for msg_id: %s", ack_id.c_str());
                //resend next
                ReSend();
            } else {
                //std::cout << "Out-of-order ack received: " << ack_id << " (ignored)\n";
                //PRINT_INFO("Out-of-order ack received: %s (ignored)", ack_id.c_str());
            }
        } else if (type == "data") {
            std::string msg_id = JsonUtils::get_json_data<std::string>(j, "msg_id", "");
            if (!msg_id.empty()) {
                const Json j_reply =
                {
                    {"type", "ack"},
                    {"msg_id", msg_id}
                };
                PRINT_DEBUG("send ack msg_id(%s)", msg_id.c_str());

                zmq::message_t zmsg(j_reply.dump().data(), j_reply.dump().size());
                m_dealer.send(zmsg, ZMQ_SEND_FLAGS_NONE);
            }
            uint32_t msg_seq_0 = JsonUtils::get_json_data<uint32_t>(j, "msg_seq_0", 0);
            uint32_t msg_seq_1 = JsonUtils::get_json_data<uint32_t>(j, "msg_seq_1", 0);
            //if use msg_seq <= m_client_msg_seq[sclient_id], when program reboot?
            uint64_t msg_seq = ((uint64_t) msg_seq_1 << 32) | (uint64_t) msg_seq_0;
            if (msg_seq == m_last_recv_msg_seq) {
                PRINT_INFO("recv msg seq(%lu) == last recv msg_seq(%lu), drop", msg_seq, m_last_recv_msg_seq);
                return true;
            }
            m_last_recv_msg_seq = msg_seq;

            //Json jcmd = JsonUtils::read_json_from_str(j);
            std::string cmd = JsonUtils::get_json_data<std::string>(j, "cmd", "");
            auto it = m_map_handler.find(cmd);
            if (it != m_map_handler.end()) {

                const std::string msg_data = JsonUtils::get_json_data<std::string>(j, "data", "");
                if (msg_data.empty()) {
                    ReliableSend( cmd, "");
                    return true;
                }
                //auto msg_data2 = zbase64::decode()
                Json jdata = JsonUtils::read_json_from_str(msg_data);
                const auto reply_msg = it->second(jdata);
                if (!reply_msg.empty()) {
                    ReliableSend(cmd, reply_msg);
                } else {
                    ReliableSend(cmd, "");
                }
            }
        }
    }
    else
    {
        return false;
    }
    return true;
}

void ZmqClient::Send(const std::string &cmd, const std::string &msg) {
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
        {"data", msg}
    };
    Send(j.dump());
}

void ZmqClient::ReliableSend(const std::string &cmd, const std::string &msg) {
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
        {"data", msg}
    };
    if(m_pending.size() > 10000)
    {
        PRINT_INFO("m_pending size(%lu) > 10000, clear", m_pending.size());
        m_pending.clear();
    }
    m_pending.push_back({msg_id, j.dump(),  0});
    ReSend();
}


void ZmqClient::Send(const std::string &msg) {
                zmq::message_t zmsg(msg.data(), msg.size());
    //PRINT_DEBUG("client send msg: %s", msg.c_str());
    m_dealer.send(zmsg, ZMQ_SEND_FLAGS_NONE);
    ReSend();
}

void ZmqClient::RegisterCmd(const std::string &cmd, std::function<std::string(const Json &j)> func) {
    m_map_handler[cmd] = func;
}

void ZmqClient::ReSend() {
    if (!m_pending.empty()) {
        PendingMessage &pmsg = m_pending.front();
        //auto now = std::chrono::steady_clock::now();
        auto elapsed = Clock::to_now_ms(pmsg.time_stamp_us);
        if (elapsed < 80 && pmsg.time_stamp_us != 0) {
            return;
        }
        //PRINT_DEBUG("Realiable Send msg_id(%s), msg_time_stamp = %.3f", pmsg.msg_id.c_str(), pmsg.content.c_str(),
        //1e-6 * pmsg.time_stamp_us);
        pmsg.time_stamp_us = Clock::time_stamp_us();
         zmq::message_t zmsg(pmsg.content.data(), pmsg.content.size());
        m_dealer.send(zmsg, ZMQ_SEND_FLAGS_NONE);
    }
}


