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
#include <iostream>
#include <vector>
#include <cstdint>

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

// One decoded frame buffered during pcap replay. Held per-driver (see
// HesaiDriver::pcap_frames_) so several drivers can replay different pcap files
// concurrently without sharing state.
struct StoredFrame
{
    std::vector<float> points;
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
      int log_server_port,
      const std::string& correctionData,
      bool pcap_parser_enabled,
      const std::string& pcap_parser_file_path
    );
    bool start();
    PointCloudFetchResult getLatestFrame(float* out_buf);
    // Pop the next decoded frame from this driver's own pcap-replay buffer.
    PointCloudFetchResult getNextPcapFrame(float* out_buf);
    bool periodicStatusThread();
    LidarStatus getStatus() const;
    void updateSnapshotDirectory(const std::string& snapshot_dir);
    void close();

private:
    // ---- Per-instance frame state (was file-scope globals in core.cpp) ----
    // The decoded-frame callback is bound to `this`, so the buffer it fills and
    // getNextPcapFrame() reads are per-driver. This is what lets several
    // HesaiDriver instances replay different pcap files at once without colliding.
    void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRT>& frame);
    PointCloudFetchResult returnLatestCloud(float* out_buf);
    void resetPcapFrameBuffer();

    float out_buf_cache_[230400 * 4] = {};   // 128 * 900 * 2 points * 4 floats
    uint64_t last_frame_time_ = 0;
    uint32_t last_frame_index_recv_ = 0;
    float dt_frame_ = 0.0f;

    bool pcap_parser_mode_ = false;
    std::vector<StoredFrame> pcap_frames_;
    uint32_t last_frame_index_sent_ = 0;

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
    std::streambuf* old_cout_buf_ = nullptr; // To restore original cout buffer if redirecting logs
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
    using SignalHandler = void(*)(int);
    SignalHandler prev_sigint_handler_ = nullptr;
    bool signal_handler_installed_ = false;
    static volatile std::sig_atomic_t sigint_received_;
    static void HesaiDriver::_sigintHandler(int);

};
