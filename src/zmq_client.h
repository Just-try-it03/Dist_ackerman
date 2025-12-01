#ifndef __ZMQ_CLIENT_H__
#define __ZMQ_CLIENT_H__
#include <string>
#include <zmq.hpp>
#include <unordered_map>
#include <deque>
#include "json.h"
#include "zmq_util.h"

class ZmqClient {
public:
    ZmqClient();

    //void Init(const std::string &addr);
    void Init(const std::string &addr, const std::string &client_id = "client-reliable");

    void Run();

    void Recv();

    void Send(const std::string &cmd, const std::string &msg);

    void ReliableSend(const std::string &cmd, const std::string &msg);

    void RegisterCmd(const std::string &cmd, std::function<std::string(const Json &j)> func);

private:
    void ReliableSend(const std::string &msg);

    void Send(const std::string &msg);

    void ReSend();

    bool RecvOneMsg();

    zmq::context_t m_context;
    zmq::socket_t m_dealer;
    //std::shared_ptr<zmq::socket_t> mp_dealer;
    std::deque<PendingMessage> m_pending;

    std::unordered_map<std::string, std::function<std::string (const Json &j)> > m_map_handler;
    uint64_t m_last_recv_msg_seq;
};

#endif // __ZMQ_CLIENT_H__
