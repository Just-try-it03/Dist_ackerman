//
// Created by ucore on 5/16/25.
//
#include "zmq_util.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <ifaddrs.h>
#include <netpacket/packet.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static uint64_t MSG_SEQ = 0;
static uint64_t MSG_ID = 0;

uint64_t GenMsgSeq() {
  ++MSG_SEQ;
  return MSG_SEQ;
}

std::string GenMsgId() {
  return std::to_string(++MSG_ID);
}

std::string GetMacAddress() {
  struct ifaddrs *ifaddr;
  if (getifaddrs(&ifaddr) == -1) return "00:00:00:00:00:00";
  std::string mac;
  for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_PACKET) {
      struct sockaddr_ll *s = (struct sockaddr_ll *) ifa->ifa_addr;
      std::ostringstream oss;
      for (int i = 0; i < s->sll_halen; ++i) {
        if (i > 0) oss << ":";
        oss << std::hex << std::setw(2) << std::setfill('0') << (int) s->sll_addr[i];
      }
      mac = oss.str();
      break;
    }
  }
  freeifaddrs(ifaddr);
  return mac;
}

std::string GetIpAddress() {
  struct ifaddrs *ifaddr;
  if (getifaddrs(&ifaddr) == -1) return "0.0.0.0";
  std::string ip;
  for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
      void *addr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;
      char buf[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, addr, buf, INET_ADDRSTRLEN);
      ip = buf;
      break;
    }
  }
  freeifaddrs(ifaddr);
  return ip;
}

std::string GetCpuId() {
  std::ifstream cpuinfo("/proc/cpuinfo");
  std::string line;
  while (std::getline(cpuinfo, line)) {
    if (line.find("Serial") != std::string::npos ||
        line.find("ID") != std::string::npos) {
      return line.substr(line.find(":") + 1);
    }
  }
  return "no-cpu-id";
}

std::string GenUuid() {
  auto now = std::chrono::system_clock::now();
  auto epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
  int pid = getpid();
  auto tid = std::this_thread::get_id();

  std::stringstream ss;
  ss << GetMacAddress() << "-"
      << GetIpAddress() << "-"
      << GetCpuId() << "-"
      << epoch << "-"
      << pid << "-"
      << tid;

  std::hash<std::string> hasher;
  size_t hash_val = hasher(ss.str());

  std::stringstream final;
  final << std::hex << hash_val;
  return final.str();
}
