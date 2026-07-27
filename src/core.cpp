#include "HesaiDriver.hpp"
#include "core.hpp"
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <csignal>

// #define LIDAR_PARSER_TEST
// #define SERIAL_PARSER_TEST
#define PCAP_PARSER_TEST
// #define EXTERNAL_INPUT_PARSER_TEST
// #define LIDAR_PARSER_TCP_TEST

// Unused legacy globals (kept as-is).
uint32_t cur_frame_time = 0;
bool running = true;
int kMaxTimeInterval = 250000;

LidarDecodedFrame<LidarPointXYZICRT> latest_frame;

// dt_logpcap stays global with the pcap-logging path. The per-frame/replay
// state (last_frame_time, indices, out_buf_cache, dt_frame, pcap_frames) moved
// into HesaiDriver so multiple drivers no longer share it.
float dt_logpcap = std::chrono::duration<float>(0.0f).count();

// pcap related variables
uint64_t t_pcap_log_start = 0.0;
std::string snapshot_directory_prev = "";
std::ofstream g_pcap_log_file;
std::ofstream g_pcap_log_file_snapshot;

// ---- PCAP format structures ----
#pragma pack(push, 1)
struct PcapGlobalHeader {
    uint32_t magic_number  = 0xa1b2c3d4; // microsecond timestamps
    uint16_t version_major = 2;
    uint16_t version_minor = 4;
    int32_t  thiszone      = 0;
    uint32_t sigfigs       = 0;
    uint32_t snaplen       = 65535;
    uint32_t network       = 1;           // LINKTYPE_ETHERNET
};

struct PcapRecordHeader {
    uint32_t ts_sec;
    uint32_t ts_usec;
    uint32_t incl_len;
    uint32_t orig_len;
};

struct EthernetHeader {
    uint8_t  dst[6] = {0xff,0xff,0xff,0xff,0xff,0xff};
    uint8_t  src[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
    uint16_t ethertype = 0x0008; // 0x0800 little-endian
};

struct IPv4Header {
    uint8_t  ver_ihl    = 0x45;
    uint8_t  dscp       = 0;
    uint16_t total_len;           // filled per packet
    uint16_t id         = 0;
    uint16_t flags_frag = 0;
    uint8_t  ttl        = 64;
    uint8_t  protocol   = 17;    // UDP
    uint16_t checksum   = 0;
    uint32_t src_ip;              // filled per packet
    uint32_t dst_ip     = 0;
};

struct UDPHeader {
    uint16_t src_port;            // filled per packet
    uint16_t dst_port;            // filled per packet
    uint16_t length;              // filled per packet
    uint16_t checksum   = 0;
};
#pragma pack(pop)

uint64_t nowtUTC() {
    std::time_t now = std::time(nullptr);
    return static_cast<uint64_t>(now);
}

std::uint64_t round_time_to_nearest_10s(double timestamp) {
    return static_cast<uint64_t>(std::round(timestamp / 10) * 10);
}

std::uint64_t nowtUTC_rounded10s() {
    uint64_t now = nowtUTC();
    return round_time_to_nearest_10s(static_cast<double>(now));
}

bool createEmptyFile(const std::string& filename) {
    std::ofstream file(filename, std::ios::trunc | std::ios::binary);
    bool success = file.good();
    return success;
}

void WritePcapGlobalHeader(std::ofstream& f) {
    PcapGlobalHeader hdr;
    f.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
}

void WritePcapPacket(std::ofstream& f, const hesai::lidar::UdpPacket& pkt,
                     uint16_t dst_port = 2368) {
    constexpr uint16_t eth_len = sizeof(EthernetHeader);
    constexpr uint16_t ip_len  = sizeof(IPv4Header);
    constexpr uint16_t udp_len = sizeof(UDPHeader);

    uint16_t payload_len    = pkt.packet_len;
    uint16_t udp_total      = udp_len + payload_len;
    uint16_t ip_total       = ip_len  + udp_total;
    uint32_t frame_len      = eth_len + ip_total;

    // Record header
    PcapRecordHeader rec;
    rec.ts_sec  = static_cast<uint32_t>(pkt.recv_timestamp / 1000000);
    rec.ts_usec = static_cast<uint32_t>(pkt.recv_timestamp % 1000000);
    rec.incl_len = frame_len;
    rec.orig_len = frame_len;
    f.write(reinterpret_cast<const char*>(&rec), sizeof(rec));

    // Ethernet
    EthernetHeader eth;
    f.write(reinterpret_cast<const char*>(&eth), sizeof(eth));

    // IP
    IPv4Header ip;
    ip.total_len = htons(ip_total);
    ip.src_ip    = pkt.ip;          // already in network byte order
    f.write(reinterpret_cast<const char*>(&ip), sizeof(ip));

    // UDP
    UDPHeader udp;
    udp.src_port = htons(pkt.port);
    udp.dst_port = htons(dst_port);
    udp.length   = htons(udp_total);
    f.write(reinterpret_cast<const char*>(&udp), sizeof(udp));

    // Payload
    f.write(reinterpret_cast<const char*>(pkt.buffer), payload_len);
}

//log info, display frame message
void HesaiDriver::lidarCallback(const LidarDecodedFrame<LidarPointXYZICRT>&frame) {
  auto t0 = std::chrono::system_clock::now();
  last_frame_time_ = nowtUTC();
  last_frame_index_recv_ = frame.frame_index;

  StoredFrame stored_frame;
  if (pcap_parser_mode_) {
      stored_frame.points.resize(230400 * 4, 0.0f);
      stored_frame.timestamp = static_cast<double>(frame.frame_end_timestamp);
  }

  // Copy point cloud data to output buffer
  size_t num_points = frame.points_num;
  for (size_t i = 0; i < num_points; ++i) {
      if (i >= 230400) break; // Prevent buffer overflow, adjust as needed

      const auto& pt = frame.points[i];
      out_buf_cache_[i * 4 + 0] = pt.x;
      out_buf_cache_[i * 4 + 1] = pt.y;
      out_buf_cache_[i * 4 + 2] = pt.z;
      out_buf_cache_[i * 4 + 3] = pt.intensity;
        if (pcap_parser_mode_) {
          stored_frame.points[i * 4 + 0] = pt.x;
          stored_frame.points[i * 4 + 1] = pt.y;
          stored_frame.points[i * 4 + 2] = pt.z;
          stored_frame.points[i * 4 + 3] = pt.intensity;
        }
  }

  if (pcap_parser_mode_) {
    pcap_frames_.push_back(std::move(stored_frame));
  }

  auto t1 = std::chrono::system_clock::now();
  dt_frame_ = std::chrono::duration<float>(t1 - t0).count();
}

PointCloudFetchResult HesaiDriver::returnLatestCloud(float* out_buf) {
    std::memcpy(out_buf, out_buf_cache_, sizeof(out_buf_cache_));
    float dt_tot = dt_frame_ + dt_logpcap;
    return {true, 0, dt_tot, static_cast<double>(last_frame_index_recv_)};
}

void HesaiDriver::resetPcapFrameBuffer() {
  pcap_frames_.clear();
  last_frame_index_sent_ = 0;
}

void faultMessageCallback(const FaultMessageInfo& fault_message_info) {
  // Use fault message messages to make some judgments
  fault_message_info.Print();
  return;
}

void rawPacketCallback(const UdpFrame_t& udp_packets, double timestamp, const std::string& logging_directory, const std::string& snapshot_directory)
{
  auto t0 = std::chrono::system_clock::now();

  uint64_t now = nowtUTC_rounded10s();
  bool create_new_pcap_file = false;
  bool create_pcap_file_snapshot = false;
  if (t_pcap_log_start == 0.0){ 
    // First time seeing a packet, start logging
      t_pcap_log_start = now;
      create_new_pcap_file = true;
      create_pcap_file_snapshot = true;
  } else { 
    // Rotate logs every 10 seconds
    if (now - t_pcap_log_start >= 10) { 
      t_pcap_log_start = now;
      create_new_pcap_file = true;
      create_pcap_file_snapshot = true;
    }
  }
  std::string file_name = std::to_string(t_pcap_log_start) + ".JT128.pcap";

  if (snapshot_directory != "" && snapshot_directory_prev == ""){
    // If snapshot directory just got set, create a new pcap file immediately regardless fo t_pcap_log_start
    create_pcap_file_snapshot = true;
  }
  
  // update previous snapshot directory
  snapshot_directory_prev = snapshot_directory;

  // create_new_pcap_file is only based on time, need to check logging directory as well
  if (create_new_pcap_file && logging_directory != "") {
    g_pcap_log_file.close();
    std::string full_path1 = logging_directory + "/" + file_name;
    std::cout << "C++: Rotating PCAP log file to: " << full_path1 << std::endl;
    bool success1 = createEmptyFile(full_path1);
    if (!success1){
      std::cout << "C++: Failed to create PCAP log file: " << full_path1 << std::endl;
    } else {
      g_pcap_log_file.open(full_path1, std::ios::binary | std::ios::out | std::ios::trunc);
      WritePcapGlobalHeader(g_pcap_log_file);
    }
  }
  if (create_pcap_file_snapshot && snapshot_directory != "") {
    g_pcap_log_file_snapshot.close();
    std::string full_path2 = snapshot_directory + "/" + file_name;
    std::cout << "C++: Rotating PCAP snapshot file to: " << full_path2 << std::endl;
    bool success2 = createEmptyFile(full_path2);
    if (!success2){
      std::cout << "C++: Failed to create PCAP snapshot log file: " << full_path2 << std::endl;
    } else {
      g_pcap_log_file_snapshot.open(full_path2, std::ios::binary | std::ios::out | std::ios::trunc);
      WritePcapGlobalHeader(g_pcap_log_file_snapshot);
    }
  }

  // -------------------------------------------------------------------------------------------------
  // LOG PACKET TO PCAP FILE(S)
  // -------------------------------------------------------------------------------------------------
  if (logging_directory != "" && g_pcap_log_file.is_open()) {
    for (const auto& packet : udp_packets) {
      WritePcapPacket(g_pcap_log_file, packet);
    }
  } else {
    g_pcap_log_file.close();
  }
  if (snapshot_directory != "" && g_pcap_log_file_snapshot.is_open()) {
    for (const auto& packet : udp_packets) {
      WritePcapPacket(g_pcap_log_file_snapshot, packet);
    }
  } else {
    g_pcap_log_file_snapshot.close();
  }

  auto t1 = std::chrono::system_clock::now();
  dt_logpcap = std::chrono::duration<float>(t1 - t0).count();
}
