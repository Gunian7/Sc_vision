#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // sockaddr_in

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
class Plotter
{
public:
  Plotter();
  explicit Plotter(const std::string & config_path);
  Plotter(const std::string & host, uint16_t port);

  ~Plotter();

  void plot(const nlohmann::json & json);

private:
  void init_destination(const std::string & host, uint16_t port);

  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP
