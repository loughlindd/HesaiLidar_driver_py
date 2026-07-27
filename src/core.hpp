#pragma once

#include "hesai_lidar_sdk.hpp"

#include <cstdint>
#include <ctime>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

// Unused legacy globals (kept as-is).
extern uint32_t cur_frame_time;
extern bool running;
extern int kMaxTimeInterval;

// dt_logpcap belongs to the (still process-global) pcap-logging path below.
extern float dt_logpcap;

// Functions implemented in the .cpp file. The per-frame/replay callbacks
// (lidarCallback / returnLatestCloud / getNextPcapFrame / resetPcapFrameBuffer)
// are now HesaiDriver members so multiple drivers don't share frame state.
uint64_t nowtUTC();

std::uint64_t nowtUTC_rounded10s();

void rawPacketCallback(const UdpFrame_t& udp_packets, double timestamp, const std::string& logging_directory, const std::string& snapshot_directory);

void faultMessageCallback(const FaultMessageInfo& fault_message_info);
