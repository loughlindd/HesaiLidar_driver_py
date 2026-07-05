// Implementation of HesaiDriver methods

#include <winsock2.h>
#include "logs_redirection.cpp"
#include "HesaiDriver.hpp"
#include "core.hpp"


volatile int HesaiDriver::sigint_received_ = 0;

HesaiDriver::HesaiDriver(){}
HesaiDriver::~HesaiDriver() { close(); }

void HesaiDriver::_sigintHandler(int)
{
    sigint_received_ = 1;
}

bool HesaiDriver::init(
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
) {
    // Ensure no stale resources from a prior session remain bound.
    close();

        // setup loggers
        if (log_server_port != 0 && log_server_ip != "") {
            redirect_logs_to_udp_ = true;
            // Create UDP buffer as member variable to maintain lifetime
            udp_buf_ = std::make_unique<UdpStreamBuf>(log_server_ip, log_server_port);
            tee_buf_ = std::make_unique<TeeStreamBuf>(std::cout.rdbuf(), udp_buf_.get());
            old_cout_buf_ = std::cout.rdbuf(tee_buf_.get());
            std::cout << "C++: Logging initialized. Redirecting logs to UDP " << log_server_ip << ":" << log_server_port << std::endl;
        }

    // Reset global frame state so stale timestamps from a previous session don't
    // trigger the connection-loss path in getLatestFrame before any frame arrives.
    last_frame_time = 0;
    last_frame_index_recv = 0;
    last_frame_index_sent = 0;
    pcap_parser_mode = pcap_parser_enabled;
    resetPcapFrameBuffer();

    // Stop and destroy the previous SDK instance before creating a fresh one.
    // The sleep gives the SDK's internal receive threads time to fully terminate
    // and the OS time to release the UDP port. Without this, a zombie receive
    // thread from the prior session can intercept incoming packets, causing the
    // new Start() to block indefinitely waiting for data that never arrives.
    if (hesai_sdk_) {
        hesai_sdk_->Stop();
        hesai_sdk_.reset();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    hesai_sdk_ = std::make_unique<HesaiLidarSdk<LidarPointXYZICRT>>();

    // Process exit flag
    exit_process_cmd_ = false;
    prev_sigint_handler_ = std::signal(SIGINT, &HesaiDriver::_sigintHandler);
    signal_handler_installed_ = true;
    
    // Save parameters
    lidar_type_ = lidar_type;
    logging_dir_ = logging_dir;
    snapshot_dir_ = "";
    lidar_address_ = lidar_address; // <--- fixed: assign host_address parameter
    host_address_ = host_address; // <--- fixed: assign host_address parameter
    ptc_port_ = ptc_port;
    scans_port_ = scans_port;
    fault_message_port_ = fault_message_port;

    current_lidar_status_ = LidarStatus();

    DriverParam param;
    param.input_param.source_type = DATA_FROM_LIDAR;
    param.input_param.device_ip_address = lidar_address_;  // lidar ip
    param.input_param.ptc_port = static_cast<uint16_t>(ptc_port_); // lidar ptc port
    param.input_param.udp_port = static_cast<uint16_t>(scans_port_); // point cloud destination port
    param.input_param.multicast_ip_address = "";

    param.input_param.ptc_mode = PtcMode::tcp;
    param.input_param.use_ptc_connected = false;  // true: use PTC connected, false: recv correction from local file
    param.input_param.correction_file_path = correctionData;
    param.input_param.firetimes_path = "Your firetime file path";

    param.input_param.host_ip_address = host_address_; // point cloud destination ip, local ip
    param.input_param.fault_message_port = static_cast<uint16_t>(fault_message_port_); // fault message destination port
    // PtcMode::tcp_ssl use
    param.input_param.certFile = "";
    param.input_param.privateKeyFile = "";
    param.input_param.caFile = "";

    param.input_param.recv_point_cloud_timeout = 0.5; // seconds
    param.input_param.ptc_connect_timeout = 0.5; // seconds

    if (pcap_parser_enabled) {
        param.input_param.source_type = DATA_FROM_PCAP;
        param.input_param.pcap_path = pcap_parser_file_path;

        param.decoder_param.pcap_play_synchronization = false;
        param.decoder_param.pcap_play_in_loop = false;
    }

    // param.input_param.source_type = DATA_FROM_PCAP;
    // param.input_param.pcap_path = "E:/aeroViz/dev/2026-04-14-ReplayHesaiPCAP/1776201541.pcap";
    // param.input_param.correction_file_path = "C:\\Users\\lough\\OneDrive - BenjaminMuylDesign\\BMDxPixel\\dev\\JT128\\JT128_default_angle.csv";

    // param.decoder_param.pcap_play_synchronization = true;
    // param.decoder_param.play_rate_ = 1.0;
    // param.decoder_param.pcap_play_in_loop = true; // pcap playback

    param.decoder_param.enable_packet_loss_tool = false;
    param.decoder_param.socket_buffer_size = 262144000;
    
    Logger::GetInstance().bindLogCallback(
        [](LOGLEVEL level, const char* file, int line, const char* func, char* msg) {
            const char* lvl = "INFO";
            if (level & HESAI_LOG_DEBUG) lvl = "DEBUG";
            else if (level & HESAI_LOG_WARNING) lvl = "WARNING";
            else if (level & HESAI_LOG_ERROR) lvl = "ERROR";
            else if (level & HESAI_LOG_FATAL) lvl = "FATAL";

            // Goes through your TeeStreamBuf -> UDP
            std::cout << "[HESAI][" << lvl << "] "
                    << msg << std::endl;
        }
    );

    std::cout << "C++: Calling Hesai SDK Init (async init thread)..." << std::endl;
    if (!hesai_sdk_->Init(param)) {
        std::cout << "C++: Driver Initialize Error..." << std::endl;
        return false;
    }
    std::cout << "C++: Hesai SDK Init returned." << std::endl;

    // int rc = hesai_sdk_->lidar_ptr_->LoadCorrectionString(correctionData.data(),
    //                                           static_cast<int>(correctionData.size()));
    // if (rc != 0) {
    //     std::cout << "C++: Correction data load failed..." << std::endl;
    //     return false;
    // }
    // std::cout << "C++: Correction data loaded successfully." << std::endl;

    hesai_sdk_->RegRecvCallback(faultMessageCallback);
    hesai_sdk_->RegRecvCallback(lidarCallback);
    hesai_sdk_->RegRecvCallback(
        [dir = std::ref(logging_dir_), snapdir = std::ref(snapshot_dir_)](const UdpFrame_t& udp_packets, double timestamp) {
            rawPacketCallback(udp_packets, timestamp, dir, snapdir);
         }
    );
    std::cout << "C++: Hesai callbacks registered." << std::endl;

    // Create driver
    // rsdriver_ = std::make_unique<LidarDriver<PointCloudT<PointXYZI>>>();

    // Register callbacks (using static or free functions for now)
    // rsdriver_->regPointCloudCallback(
    //     driverGetPointCloudFromCallerCallback,
    //     driverReturnPointCloudToCallerCallback
    // );
    // rsdriver_->regExceptionCallback([](const Error& code) {
    //     RS_WARNING << code.toString() << std::endl;
    // });
    // rsdriver_->regPacketCallback(
    //     [dir = logging_dir_, snapdir = std::ref(snapshot_dir_)](const robosense::lidar::Packet& pkt) {
    //         logLidarPacket(pkt, dir, snapdir);
    // });

    return true;
}

bool HesaiDriver::start() {
    hesai_sdk_->Start();  // waits until ready or fail
    if (hesai_sdk_->lidar_ptr_ && hesai_sdk_->lidar_ptr_->GetInitFinish(FailInit)) {
        std::cout << "C++: Hesai Driver start failed (init timeout or init error)." << std::endl;
        return false; // teardown handled by next init() via make_unique reset
    }

    std::cout << "C++: Hesai Driver started successfully!" << std::endl;
    return true;

    // periodic_status_thread_ = std::thread(&HesaiDriver::periodicStatusThread, this);

    // bool startedOK = rsdriver_->start();
    // if (!startedOK) {
    //     RS_ERROR << "C++: Driver Start Error..." << std::endl;
    //     return false;
    // }

    // Dedicated SIGINT monitoring thread
    // sig_monitor_thread_ = std::thread([this]() {
    //     while (!exit_process_cmd_) {
    //         if (sigint_received_) {
    //             exit_process_cmd_.store(true);
    //             break;
    //         }
    //         std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //     }
    // });
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    // Try to get latest frame, to ensure lidar is properly connected
    // double* dummy_buf = new double[900*96*4]; // Adjust size as needed
    // PointCloudFetchResult result = returnLatestCloud(dummy_buf);
    // bool frameOK = result.success;

    // if (frameOK) {
    //     std::cout << "C++: Driver & Threads successfully started !" << std::endl;
    // }
}

PointCloudFetchResult HesaiDriver::getLatestFrame(float* out_buf) {
    uint64_t now = nowtUTC();
    if (last_frame_time != 0 && now - last_frame_time > 5) {
        return {false, 0.0f, 0.0f, 0.0f};
    }
    PointCloudFetchResult result = returnLatestCloud(out_buf);
    return result;
}

PointCloudFetchResult getNextPcapFrame(float* out_buf) {
    if (last_frame_index_sent >= pcap_frames.size()) {
        return {false, 0.0f, 0.0f, 0.0f};
    }

    const StoredFrame& frame = pcap_frames[last_frame_index_sent];
    std::memcpy(out_buf, frame.points.data(), frame.points.size() * sizeof(float));
    last_frame_index_sent += 1;

    float dt_tot = dt_frame + dt_logpcap;
    return {true, 0.0f, dt_tot, frame.timestamp};
}

// Return the LidarStatus object so the wrapper can handle conversion to Python.
LidarStatus HesaiDriver::getStatus() const {
    return current_lidar_status_;
}

void HesaiDriver::updateSnapshotDirectory(const std::string& snapshot_dir) {
    snapshot_dir_ = snapshot_dir;
    // std::cout << "C++: Updated snapshot directory to: " << snapshot_dir_ << std::endl;
}

bool HesaiDriver::periodicStatusThread() {
    // while (!exit_process_cmd_) {
    //     float temp;
    //     DeviceInfo info;
    //     DeviceStatus status;
    //     bool temp_ok = rsdriver_->getTemperature(temp);
    //     bool info_ok = rsdriver_->getDeviceInfo(info);
    //     bool status_ok = rsdriver_->getDeviceStatus(status);

    //     if (temp_ok) {
    //         current_lidar_status_.temperature = temp;
    //     }
    //     if (info_ok && info.state){
    //         std::string sn_str;
    //         for (size_t i = 0; i < sizeof(info.sn); ++i) {
    //             char buf[3];
    //             std::sprintf(buf, "%02X", info.sn[i]);
    //             sn_str += buf;
    //         }
    //         std::string mac_str;
    //         for (size_t i = 0; i < sizeof(info.mac); ++i) {
    //             char buf[3];
    //             std::sprintf(buf, "%02X", info.mac[i]);
    //             mac_str += buf;
    //             if (i < sizeof(info.mac) - 1) mac_str += ":";
    //         }
    //         std::string top_fw_ver_str;
    //         for (size_t i = 0; i < sizeof(info.top_ver); ++i) {
    //             char buf[3];
    //             std::sprintf(buf, "%02X", static_cast<unsigned int>(info.top_ver[i]));
    //             top_fw_ver_str += buf;
    //         }
    //         std::string bottom_fw_ver_str;
    //         for (size_t i = 0; i < sizeof(info.bottom_ver); ++i) {
    //             char buf[3];
    //             std::sprintf(buf, "%02X", static_cast<unsigned int>(info.bottom_ver[i]));
    //             bottom_fw_ver_str += buf;
    //         }
    //         current_lidar_status_.sn = sn_str;
    //         current_lidar_status_.mac = mac_str;
    //         current_lidar_status_.top_fw_ver = top_fw_ver_str;
    //         current_lidar_status_.bottom_fw_ver = bottom_fw_ver_str;
    //     }
    //     if (status_ok && status.state){
    //         current_lidar_status_.voltage = status.voltage/100; // make V not cV
    //     }
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }
    return true;
}

void HesaiDriver::close() {
    // Idempotent shutdown: safe if called multiple times from Python and dtor.
    if (hesai_sdk_) {
        hesai_sdk_->Stop();
        hesai_sdk_.reset(); // null so a subsequent init() doesn't double-stop
    }
    exit_process_cmd_.store(true);
    

    // rsdriver_->stop();

    if (pcap_thread_.joinable()) pcap_thread_.join();
    // if (periodic_status_thread_.joinable()) periodic_status_thread_.join();
    // if (sig_monitor_thread_.joinable()) sig_monitor_thread_.join();
    
    if (redirect_logs_to_udp_ && old_cout_buf_ != nullptr) {
        std::cout.rdbuf(old_cout_buf_);
        old_cout_buf_ = nullptr;
    }
    tee_buf_.reset();
    udp_buf_.reset();
    redirect_logs_to_udp_ = false;

    if (signal_handler_installed_) {
        std::signal(SIGINT, prev_sigint_handler_);
        signal_handler_installed_ = false;
        prev_sigint_handler_ = nullptr;
    }

    std::cout << "C++: KeyboardInterrupt: Stopping processes and drivers..." << std::endl;
}
