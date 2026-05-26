// RobosenseDriver.hpp
// C++ class wrapper for RoboSense point cloud driver logic
// This is a skeleton based on suggestions, not yet integrated with existing code
#define WIN32_LEAN_AND_MEAN

#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <csignal>
#include <windows.h>

#include "hesai_lidar_sdk.hpp"

// Forward declarations for stream buffer classes from logs_redirection.cpp
class UdpStreamBuf;
class TeeStreamBuf;

struct PointCloudFetchResult
{
    bool success;
    float dt_wait;
    float dt_proc;
    double timestamp;
};

struct LidarStatus
{
  LidarStatus(){}
    float temperature = 0.0f;
    //
    std::string sn = "";
    std::string mac = "";
    std::string top_fw_ver = "";
    std::string bottom_fw_ver = "";
    //
    float voltage = 0.0f;
  };


class HesaiDriver {
public:
    HesaiDriver();
    ~HesaiDriver();

    // Initialize the driver
    bool init(
      int lidar_type, 
      const std::string& lidar_address, 
      const std::string& host_address, 
      int scans_port, 
      int ptc_port, 
      int fault_message_port, 
      const std::string& logging_dir, 
      const std::string& log_server_ip, 
      int log_server_port);
    bool start();
    PointCloudFetchResult getLatestFrame(double* out_buf);
    bool periodicStatusThread();
    LidarStatus getStatus() const;
    void updateSnapshotDirectory(const std::string& snapshot_dir);
    void close();

private:
    // Parameters
    int lidar_type_;
    std::string host_address_;
    std::string lidar_address_;
    int scans_port_;
    int ptc_port_;
    int fault_message_port_;
    std::string logging_dir_;
    std::string snapshot_dir_;
    
    std::string log_server_ip_;
    int log_server_port_;

    bool redirect_logs_to_udp_ = false;
    std::streambuf* old_cout_buf_; // To restore original cout buffer if redirecting logs
    std::unique_ptr<UdpStreamBuf> udp_buf_; // UDP stream buffer for log redirection
    std::unique_ptr<TeeStreamBuf> tee_buf_;  // Tee stream buffer to split output to console and UDP

    // Status to store latest readings
    LidarStatus current_lidar_status_;

    // Recreated on each init() so Stop()->Init()->Start() always gets a clean object
    std::unique_ptr<HesaiLidarSdk<LidarPointXYZICRT>> hesai_sdk_;

    // Driver and resources
    // std::unique_ptr<LidarDriver<PointCloudT<PointXYZI>>> rsdriver_;
    std::thread pcap_thread_;
    std::thread periodic_status_thread_;
    std::thread sig_monitor_thread_;

    // Sigint handling
    std::atomic<bool> exit_process_cmd_;
    static volatile std::sig_atomic_t sigint_received_;
    static void HesaiDriver::_sigintHandler(int);

};
