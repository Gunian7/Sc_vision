#ifndef IO__HERO_CONFIG_PATH_HPP
#define IO__HERO_CONFIG_PATH_HPP

#include <filesystem>
#include <string>

namespace io
{

/**
 * 解析 YAML 配置路径：若相对当前工作目录可读则原样返回；
 * 否则若 path 为相对路径，则尝试「可执行文件目录/../path」（典型：build 下运行 → Sc_vision/configs/...）。
 */
inline std::string resolve_config_path_next_to_build(const std::string &path, const char *argv0)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path p(path);
  if (!path.empty() && fs::exists(p, ec) && fs::is_regular_file(p, ec))
  {
    return path;
  }
  if (path.empty() || p.is_absolute() || argv0 == nullptr || argv0[0] == '\0')
  {
    return path;
  }
  ec.clear();
  fs::path exe = fs::absolute(fs::path(argv0), ec);
  if (ec)
  {
    return path;
  }
  exe = fs::weakly_canonical(exe, ec);
  if (ec)
  {
    return path;
  }
  const fs::path candidate = fs::weakly_canonical(exe.parent_path() / ".." / path, ec);
  if (!ec && fs::exists(candidate) && fs::is_regular_file(candidate))
  {
    return candidate.string();
  }
  return path;
}

} // namespace io

#endif
