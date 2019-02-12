/**
 * Gets information about cpu usage from /proc/stat
 * Modified from
 * https://github.com/vivaladav/BitsOfBytes/blob/master/cpp-program-to-get-cpu-usage-from-command-line-in-linux/main.cpp
 */
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

constexpr int num_cpu_states = 10;

enum Cpu_states
{
  S_USER = 0,
  S_NICE,
  S_SYSTEM,
  S_IDLE,
  S_IOWAIT,
  S_IRQ,
  S_SOFTIRQ,
  S_STEAL,
  S_GUEST,
  S_GUEST_NICE
};

typedef struct Cpu_data
{
  std::string cpu;
  size_t times[num_cpu_states];
} CPUData;

void ReadStatsCPU(std::vector<CPUData>& entries);

size_t GetIdleTime(const CPUData& e);
size_t GetActiveTime(const CPUData& e);

void PrintStats(const std::vector<CPUData>& entries1, const std::vector<CPUData>& entries2);

/**
 * Returns total CPU usage, as well as per core usage. Each entry in usages corresponds to the individual
 * cores
 * @param[out] usages vector to return
 */
double get_usage(std::vector<double>& usages)
{
  std::vector<CPUData> entries1;
  std::vector<CPUData> entries2;
  ReadStatsCPU(entries1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ReadStatsCPU(entries2);

  const size_t num_entries = entries1.size();
  double total_usage = 0;

  for (size_t i = 0; i < num_entries; ++i)
  {
    const CPUData& e1 = entries1[i];
    const CPUData& e2 = entries2[i];

    //    std::cout.width(3);
    //    std::cout << e1.cpu << "] ";

    const auto ACTIVE_TIME = static_cast<double>(GetActiveTime(e2) - GetActiveTime(e1));
    const auto IDLE_TIME = static_cast<double>(GetIdleTime(e2) - GetIdleTime(e1));
    const auto TOTAL_TIME = ACTIVE_TIME + IDLE_TIME;

    if (i == 0)
    {
      total_usage = ACTIVE_TIME / TOTAL_TIME;
    }
    else
    {
      usages.emplace_back(ACTIVE_TIME / TOTAL_TIME);
    }
  }
  return total_usage;
}

void ReadStatsCPU(std::vector<CPUData>& entries)
{
  std::ifstream fileStat("/proc/stat");

  std::string line;

  const std::string STR_CPU("cpu");
  const std::size_t LEN_STR_CPU = STR_CPU.size();
  const std::string STR_TOT("tot");

  while (std::getline(fileStat, line))
  {
    // cpu stats line found
    if (!line.compare(0, LEN_STR_CPU, STR_CPU))
    {
      std::istringstream ss(line);

      // store entry
      entries.emplace_back(CPUData());
      CPUData& entry = entries.back();

      // read cpu label
      ss >> entry.cpu;

      // remove "cpu" from the label when it's a processor number
      if (entry.cpu.size() > LEN_STR_CPU)
        entry.cpu.erase(0, LEN_STR_CPU);
      // replace "cpu" with "tot" when it's total values
      else
        entry.cpu = STR_TOT;

      // read times
      for (int i = 0; i < num_cpu_states; ++i)
        ss >> entry.times[i];
    }
  }
}

size_t GetIdleTime(const CPUData& e)
{
  return e.times[S_IDLE] + e.times[S_IOWAIT];
}

size_t GetActiveTime(const CPUData& e)
{
  return e.times[S_USER] + e.times[S_NICE] + e.times[S_SYSTEM] + e.times[S_IRQ] + e.times[S_SOFTIRQ] +
         e.times[S_STEAL] + e.times[S_GUEST] + e.times[S_GUEST_NICE];
}
