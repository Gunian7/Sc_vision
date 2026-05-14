#ifndef TOOLS__RESOLVE_PATH_RELATIVE_TO_CONFIG_HPP
#define TOOLS__RESOLVE_PATH_RELATIVE_TO_CONFIG_HPP

#include <filesystem>
#include <string>

namespace tools
{

/**
 * 将 yaml 中的相对路径解析为可读绝对路径：
 * 先尝试相对当前工作目录；再沿配置文件所在目录逐级向上查找（直至仓库根），
 * 以便在仓库根目录启动而配置在 Sc_vision/configs 下时仍能定位 Sc_vision/assets 等目录。
 */
inline std::string resolve_path_relative_to_config(
    const std::string &config_path,
    const std::string &path_from_yaml)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  if (path_from_yaml.empty())
  {
    return path_from_yaml;
  }

  const fs::path rel(path_from_yaml);

  if (rel.is_absolute())
  {
    if (fs::exists(rel, ec) && fs::is_regular_file(rel, ec))
    {
      ec.clear();
      fs::path c = fs::weakly_canonical(rel, ec);
      return ec ? path_from_yaml : c.string();
    }
    return path_from_yaml;
  }

  const fs::path cwd_candidate = fs::current_path(ec) / rel;
  ec.clear();
  if (fs::exists(cwd_candidate, ec) && fs::is_regular_file(cwd_candidate, ec))
  {
    ec.clear();
    fs::path c = fs::weakly_canonical(cwd_candidate, ec);
    return ec ? cwd_candidate.string() : c.string();
  }

  fs::path cfg(config_path);
  ec.clear();
  if (!cfg.is_absolute())
  {
    cfg = fs::weakly_canonical(fs::current_path() / cfg, ec);
  }
  else
  {
    cfg = fs::weakly_canonical(cfg, ec);
  }

  if (!ec && !cfg.empty())
  {
    fs::path dir = cfg.parent_path();
    for (int i = 0; i < 8 && !dir.empty(); ++i)
    {
      const fs::path candidate = dir / rel;
      ec.clear();
      if (fs::exists(candidate, ec) && fs::is_regular_file(candidate, ec))
      {
        ec.clear();
        fs::path c = fs::weakly_canonical(candidate, ec);
        return ec ? candidate.string() : c.string();
      }
      dir = dir.parent_path();
    }
  }

  return path_from_yaml;
}

} // namespace tools

#endif
