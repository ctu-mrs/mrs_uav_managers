#include <mrs_uav_managers/diagnostics_manager/utils/host_stats.hpp>

namespace mrs_uav_managers::diagnostics_manager::utils
{

namespace
{

std::vector<std::string> splitByChar(const std::string &line, char delim) {
  std::vector<std::string> out;
  std::stringstream        ss(line);
  std::string              item;
  while (std::getline(ss, item, delim)) {
    out.push_back(item);
  }
  return out;
}

// Returns true if /proc/<pid>/maps contains a librclcpp.so mapping —
// a reliable signal that the process linked the ROS2 C++ client library.
// Returns false on EACCES (other-user process) or any read error.
bool isRosProcess(int pid) {
  std::ifstream maps("/proc/" + std::to_string(pid) + "/maps");
  if (!maps.is_open()) {
    return false;
  }
  std::string line;
  while (std::getline(maps, line)) {
    if (line.find("librclcpp.so") != std::string::npos) {
      return true;
    }
  }
  return false;
}

// Reads utime + stime (jiffies) from /proc/<pid>/stat. Returns -1 on failure.
// The comm field (parens-wrapped) can contain spaces and parens, so we
// anchor parsing on the final ')' rather than tokenizing the whole line.
long readProcTicks(int pid) {
  std::ifstream stat("/proc/" + std::to_string(pid) + "/stat");
  std::string   line;
  if (!std::getline(stat, line)) {
    return -1;
  }
  const auto rparen = line.rfind(')');
  if (rparen == std::string::npos || rparen + 2 > line.size()) {
    return -1;
  }
  // Tokens after ')' begin at field 3 (state); utime is field 14 → index 11, stime → index 12.
  const auto tokens = splitByChar(line.substr(rparen + 2), ' ');
  if (tokens.size() < 13) {
    return -1;
  }
  try {
    return std::stol(tokens[11]) + std::stol(tokens[12]);
  }
  catch (const std::exception &) {
    return -1;
  }
}

// Best-effort process label. Prefers __node:=<name> from /proc/<pid>/cmdline
// (so we get the actual ROS node name instead of "component_container_mt"),
// falls back to /proc/<pid>/comm. Returns "pid<N>" if both fail.
std::string readProcName(int pid) {
  std::ifstream cmdline("/proc/" + std::to_string(pid) + "/cmdline");
  if (cmdline.is_open()) {
    std::string raw((std::istreambuf_iterator<char>(cmdline)), std::istreambuf_iterator<char>());
    // cmdline tokens are NUL-separated.
    const std::string marker = "__node:=";
    const auto        pos    = raw.find(marker);
    if (pos != std::string::npos) {
      const auto end = raw.find('\0', pos);
      return raw.substr(pos + marker.size(), end - pos - marker.size());
    }
  }
  std::ifstream comm("/proc/" + std::to_string(pid) + "/comm");
  std::string   name;
  if (std::getline(comm, name) && !name.empty()) {
    return name;
  }
  return "pid" + std::to_string(pid);
}

bool isPidString(const std::string &name) {
  return !name.empty() && std::all_of(name.begin(), name.end(), [](unsigned char c) { return std::isdigit(c); });
}
} // namespace

void HostStats::setWifiInterface(std::string iface) {
  std::scoped_lock lock(mutex_);
  wifi_interface_filter_ = std::move(iface);
}

void HostStats::setNodeCpuPeriods(std::chrono::milliseconds node_cpu_sample_period, std::chrono::milliseconds pid_discovery_period) {
  std::scoped_lock lock(mutex_);
  node_cpu_sample_period_ = node_cpu_sample_period;
  pid_discovery_period_   = pid_discovery_period;
}

void HostStats::update() {
  std::scoped_lock lock(mutex_);
  readCpuFreq();
  readCpuLoad();
  readCpuTemperature();
  readMemLoad();
  readDiskSpace();
  readNodeCpuLoads();
  readWifi();
}

HostStats::Snapshot HostStats::snapshot() const {
  std::scoped_lock lock(mutex_);
  return snap_;
}

void HostStats::readCpuLoad() {
  std::ifstream file("/proc/stat");
  std::string   line;
  if (!std::getline(file, line)) {
    return;
  }

  const auto results = splitByChar(line, ' ');

  // /proc/stat format: "cpu  user nice system idle iowait irq softirq steal ..."
  // splitByChar produces an extra empty token from the double-space after "cpu",
  // so each value is shifted by one: index 0 = "cpu", 1 = "", 2 = user, etc.
  constexpr size_t kUser    = 2;
  constexpr size_t kNice    = 3;
  constexpr size_t kSystem  = 4;
  constexpr size_t kIdle    = 5;
  constexpr size_t kIowait  = 6;
  constexpr size_t kIrq     = 7;
  constexpr size_t kSoftirq = 8;
  constexpr size_t kSteal   = 9;

  long idle     = 0;
  long non_idle = 0;
  long total    = 0;

  try {
    idle     = std::stol(results.at(kIdle)) + std::stol(results.at(kIowait));
    non_idle = std::stol(results.at(kUser)) + std::stol(results.at(kNice)) + std::stol(results.at(kSystem)) + std::stol(results.at(kIrq)) +
               std::stol(results.at(kSoftirq)) + std::stol(results.at(kSteal));
    total = idle + non_idle;
  }
  catch (const std::exception &) {
    return;
  }

  cpu_last_total_diff_ = cpu_total_diff_;
  cpu_total_diff_      = total - cpu_last_total_;
  const long idle_diff = idle - cpu_last_idle_;

  if (cpu_total_diff_ > 0) {
    snap_.cpu_load = 100.0f * static_cast<float>(cpu_total_diff_ - idle_diff) / static_cast<float>(cpu_total_diff_);
  }

  cpu_last_total_ = total;
  cpu_last_idle_  = idle;
}

void HostStats::readCpuTemperature() {
  long max_temp = 0;
  for (int i = 0; i < 11; ++i) {
    const std::string path = "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/temp";
    if (!std::filesystem::exists(path)) {
      break;
    }
    std::ifstream file(path);
    std::string   line;
    if (!std::getline(file, line)) {
      continue;
    }
    try {
      const long t = std::stol(line);
      if (t > max_temp) {
        max_temp = t;
      }
    }
    catch (const std::exception &) {
      continue;
    }
  }
  snap_.cpu_temperature = static_cast<float>(max_temp) / 1000.0f;
}

void HostStats::readCpuCoreCount() {
  // /sys/devices/system/cpu/online contains a range like "0-7" for 8 cores.
  std::ifstream online_file("/sys/devices/system/cpu/online");
  std::string   online_line;
  if (!std::getline(online_file, online_line)) {
    return;
  }
  const auto parts = splitByChar(online_line, '-');
  if (parts.size() >= 2) {
    try {
      cpu_cores_ = std::stoi(parts.at(1)) + 1;
    }
    catch (const std::exception &) {
      cpu_cores_ = 1;
    }
  }
}

void HostStats::readCpuFreq() {
  readCpuCoreCount();

  long cpu_freq_khz = 0;
  for (int i = 0; i < cpu_cores_; ++i) {
    const std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_cur_freq";
    std::ifstream     file(path);
    std::string       line;
    if (!std::getline(file, line)) {
      continue;
    }
    try {
      cpu_freq_khz += std::stol(line);
    }
    catch (const std::exception &) {
      continue;
    }
  }

  if (cpu_cores_ > 0) {
    // scaling_cur_freq is in kHz; convert average across all cores to GHz.
    snap_.cpu_ghz = (static_cast<float>(cpu_freq_khz) / static_cast<float>(cpu_cores_)) / 1000000.0f;
  }
}

void HostStats::readMemLoad() {
  // /proc/meminfo first three lines: MemTotal, MemFree, MemAvailable.
  // MemAvailable already accounts for reclaimable page cache, so it is the
  // correct "usable free" figure. MemFree is read only to advance the stream.
  std::ifstream file("/proc/meminfo");
  std::string   line_total, line_mem_free, line_available;
  if (!(std::getline(file, line_total) && std::getline(file, line_mem_free) && std::getline(file, line_available))) {
    return;
  }

  auto parse_kib = [](const std::string &line) -> double {
    const auto results = splitByChar(line, ' ');
    for (size_t i = 1; i < results.size(); ++i) {
      if (!results[i].empty() && std::isdigit(results[i].front())) {
        try {
          return static_cast<double>(std::stol(results[i])) / 1048576.0; // KiB -> GiB
        }
        catch (const std::exception &) {
          return 0.0;
        }
      }
    }
    return 0.0;
  };

  snap_.total_ram = static_cast<float>(parse_kib(line_total));
  snap_.free_ram  = static_cast<float>(parse_kib(line_available));
}

void HostStats::readDiskSpace() {
  try {
    const auto si  = std::filesystem::space(".");
    snap_.free_hdd = static_cast<int32_t>(std::round(static_cast<double>(si.available) / 1073741824.0));
  }
  catch (const std::exception &) {
    snap_.free_hdd = -1;
  }
}

void HostStats::discoverRosPids(std::chrono::steady_clock::time_point now) {
  std::unordered_set<int> live_pids;
  std::unordered_set<int> discovered_ros_pids;

  std::error_code ec;
  for (const auto &entry : std::filesystem::directory_iterator("/proc", ec)) {
    if (ec) {
      break;
    }
    const std::string name = entry.path().filename().string();
    if (!isPidString(name)) {
      continue;
    }

    int pid;
    try {
      pid = std::stoi(name);
    }
    catch (const std::exception &) {
      continue;
    }
    live_pids.insert(pid);

    // Use cached ROS classification if available; otherwise probe and cache.
    auto cache_it = pid_is_ros_.find(pid);
    bool is_ros   = (cache_it != pid_is_ros_.end()) ? cache_it->second : isRosProcess(pid);
    pid_is_ros_.emplace(pid, is_ros);
    if (!is_ros) {
      continue;
    }

    discovered_ros_pids.insert(pid);
    if (pid_name_cache_.find(pid) == pid_name_cache_.end()) {
      pid_name_cache_.emplace(pid, readProcName(pid));
    }
  }

  ros_pids_ = std::move(discovered_ros_pids);

  // Evict caches for PIDs that no longer exist.
  auto evict_dead = [&live_pids](auto &map) {
    for (auto it = map.begin(); it != map.end();) {
      it = live_pids.count(it->first) ? std::next(it) : map.erase(it);
    }
  };
  evict_dead(pid_is_ros_);
  evict_dead(proc_last_ticks_);
  evict_dead(pid_name_cache_);

  last_pid_discovery_tp_ = now;
}

void HostStats::sampleNodeCpuLoads(std::chrono::steady_clock::time_point now) {
  if (node_cpu_total_diff_accum_ <= 0 || cpu_cores_ <= 0) {
    last_node_cpu_sample_tp_ = now;
    return;
  }

  std::unordered_map<int, long>       new_proc_ticks;
  std::vector<mrs_msgs::msg::CpuLoad> loads;
  std::vector<int>                    stale_ros_pids;

  // Composable-node note: nodes sharing a container share one PID, so per-PID load
  // aggregates them. The __node:= label in readProcName() falls back to comm for
  // multi-node containers — fine for operator-facing "which process is hot" views.
  for (const int pid : ros_pids_) {
    const long ticks = readProcTicks(pid);
    if (ticks < 0) {
      pid_is_ros_.erase(pid);
      pid_name_cache_.erase(pid);
      stale_ros_pids.push_back(pid);
      continue;
    }
    new_proc_ticks[pid] = ticks;

    const auto last_it = proc_last_ticks_.find(pid);
    if (last_it == proc_last_ticks_.end()) {
      continue; // No previous sample yet; wait for the next interval.
    }

    const long diff = ticks - last_it->second;
    if (diff <= 0) {
      if (diff < 0) {
        pid_is_ros_.erase(pid);
        pid_name_cache_.erase(pid);
        stale_ros_pids.push_back(pid);
      }
      continue;
    }

    const float load_pct = 100.0f * static_cast<float>(diff) * static_cast<float>(cpu_cores_) / static_cast<float>(node_cpu_total_diff_accum_);

    auto name_it = pid_name_cache_.find(pid);
    if (name_it == pid_name_cache_.end()) {
      name_it = pid_name_cache_.emplace(pid, readProcName(pid)).first;
    }

    mrs_msgs::msg::CpuLoad msg;
    msg.node_name = name_it->second;
    msg.cpu_load  = load_pct;
    loads.push_back(std::move(msg));
  }

  proc_last_ticks_ = std::move(new_proc_ticks);
  for (const int stale_pid : stale_ros_pids) {
    ros_pids_.erase(stale_pid);
  }
  snap_.node_cpu_loads       = std::move(loads);
  node_cpu_total_diff_accum_ = 0;
  last_node_cpu_sample_tp_   = now;
}

void HostStats::readNodeCpuLoads() {
  if (cpu_total_diff_ > 0) {
    node_cpu_total_diff_accum_ += cpu_total_diff_;
  }

  const auto now = std::chrono::steady_clock::now();

  const bool should_discover = !node_cpu_initialized_ || (now - last_pid_discovery_tp_ >= pid_discovery_period_);
  const bool should_sample   = !node_cpu_initialized_ || (now - last_node_cpu_sample_tp_ >= node_cpu_sample_period_);

  if (should_discover) {
    discoverRosPids(now);
  }

  if (should_sample) {
    sampleNodeCpuLoads(now);
  }

  node_cpu_initialized_ = true;
}

void HostStats::readWifi() {
  // Reset to "unavailable" sentinels; only populate on a successful read.
  snap_.wifi_interface.clear();
  snap_.wifi_signal_dbm   = std::numeric_limits<float>::quiet_NaN();
  snap_.wifi_link_quality = -1;

  std::ifstream file("/proc/net/wireless");
  if (!file.is_open()) {
    return;
  }

  std::string line;
  // Skip 2 header lines.
  std::getline(file, line);
  std::getline(file, line);

  while (std::getline(file, line)) {
    char  iface_buf[64] = {};
    int   status = 0, link = 0;
    float level = 0.0f, noise = 0.0f;

    if (std::sscanf(line.c_str(), "%63s %d %d. %f. %f.", iface_buf, &status, &link, &level, &noise) < 4) {
      continue;
    }

    std::string iface(iface_buf);
    if (!iface.empty() && iface.back() == ':') {
      iface.pop_back();
    }

    // If a specific interface is configured, only match that one.
    if (!wifi_interface_filter_.empty() && iface != wifi_interface_filter_) {
      continue;
    }

    snap_.wifi_interface    = std::move(iface);
    snap_.wifi_signal_dbm   = level;
    snap_.wifi_link_quality = link;
    return;
  }
}

} // namespace mrs_uav_managers::diagnostics_manager::utils
