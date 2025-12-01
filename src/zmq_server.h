//
// Created by ucore on 5/14/25.
//

#ifndef __ZMQ_SERVER_H__
#define __ZMQ_SERVER_H__
#include <string>
#include <zmq.hpp>
#include <unordered_map>
#include <deque>
#include "json.h"
#include "zmq_util.h"

class ZmqServer {
public:
    ZmqServer();

    void Init(const std::string &addr);

    void Run();

    void Recv();

    void Send(const std::string client_id, const std::string &cmd, const std::string &msg);

    void ReliableSend(const std::string client_id, const std::string &cmd, const std::string &msg);

    void RegisterCmd(const std::string &cmd, std::function<std::string(const Json &j)> func);

private:
    void Send(const std::string client_id, const std::string &msg);

    void ReSend(const std::string client_id);

    void ReSend();

    bool RecvOneMsg();
private:
    zmq::context_t m_context;
    //zmq::socket_t m_router;修改为：
    std::vector<std::shared_ptr<zmq::socket_t>> m_routers;  // 支持多个socket
    //std::shared_ptr<zmq::socket_t> mp_router;
    std::unordered_map<std::string, std::deque<PendingMessage> > m_pending_map;
    std::unordered_map<std::string, uint64_t> m_client_msg_seq;

    std::unordered_map<std::string, std::function<std::string(const Json &j)> > m_map_handler;
};


#endif //__ZMQ_SERVER_H__
