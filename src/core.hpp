#pragma once

#include "hesai_lidar_sdk.hpp"

#include <cstdint>
#include <ctime>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

struct StoredFrame
{
	std::vector<float> points;
	double timestamp;
};

// Global variables defined in the .cpp file
extern uint64_t last_frame_time;
extern uint32_t last_frame_index_recv;
extern uint32_t last_frame_index_sent;
extern bool pcap_parser_mode;
extern std::vector<StoredFrame> pcap_frames;
extern float dt_logpcap;
extern float dt_frame;

extern uint32_t cur_frame_time;
extern bool running;
extern int kMaxTimeInterval;

// Functions implemented in the .cpp file
uint64_t nowtUTC();

std::uint64_t nowtUTC_rounded10s();

void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRT>& frame);

void rawPacketCallback(const UdpFrame_t& udp_packets, double timestamp, const std::string& logging_directory, const std::string& snapshot_directory);

void faultMessageCallback(const FaultMessageInfo& fault_message_info);

PointCloudFetchResult returnLatestCloud(float* out_buf);

PointCloudFetchResult getNextPcapFrame(float* out_buf);

void resetPcapFrameBuffer();