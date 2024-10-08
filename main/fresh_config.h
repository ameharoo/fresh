#pragma once

#include "types.h"

// affinity for tasks meaning use all cores
const u64 NO_AFFINITY = -1;


// interface secure session establishment state
const u64 MAX_SECURE_EST_SESSION_LIVE_TIME = 4'000'000; // in microseconds

// TASK: check packets cache
const auto CHECK_PACKETS_TASK_NAME = "fresh if check packets";
const int CHECK_PACKETS_TASK_STACK_SIZE = 4096;
const int CHECK_PACKETS_TASK_PRIORITY = -7;
const u64 CHECK_PACKETS_TASK_AFFINITY = NO_AFFINITY;

// TASK: handle data packets
const auto HANDLE_DATA_PACKET_NAME = "handle mesh packet";
const int HANDLE_PACKET_TASK_STACK_SIZE = 8192;
const int HANDLE_PACKET_TASK_PRIORITY = -9;
const u64 HANDLE_PACKET_TASK_AFFINITY = NO_AFFINITY;

// controller
const int CONTROLLER_DEFAULT_PACKET_TTL = 5;

// mesh data streams
const int DATA_STREAM_RECV_PAIR_CNT = 8;
const u64 DATA_STREAM_MAX_PACKET_WAIT = 3'000'000;     // in microseconds
const u64 DATA_STREAM_BROADCAST_KEEP_TIME = 5'000'000; // in microseconds

// packet cache
const u64 MAX_RX_CACHE_ENTRY_LIVE_TIME = 5'000'000;    // in microseconds
