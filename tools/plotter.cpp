#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close
#include <yaml-cpp/yaml.h>

namespace
{
constexpr const char * DEFAULT_PLOTTER_HOST = "127.0.0.1";
constexpr uint16_t     DEFAULT_PLOTTER_PORT = 9870;
}  // namespace

namespace tools
{
Plotter::Plotter() : Plotter(DEFAULT_PLOTTER_HOST, DEFAULT_PLOTTER_PORT) {}

Plotter::Plotter(const std::string & config_path)
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  std::string host = DEFAULT_PLOTTER_HOST;
  uint16_t    port = DEFAULT_PLOTTER_PORT;

  try {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["plotter_host"].IsDefined()) {
      host = yaml["plotter_host"].as<std::string>();
    }
    if (yaml["plotter_port"].IsDefined()) {
      port = yaml["plotter_port"].as<uint16_t>();
    }
  } catch (const YAML::Exception &) {}

  init_destination(host, port);
}

Plotter::Plotter(const std::string & host, uint16_t port)
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  init_destination(host, port);
}

void Plotter::init_destination(const std::string & host, uint16_t port)
{
  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port);
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

Plotter::~Plotter() { ::close(socket_); }

void Plotter::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = json.dump();
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

}  // namespace tools
