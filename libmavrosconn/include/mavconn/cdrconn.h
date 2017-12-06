
#include <boost/system/system_error.hpp>

#include <deque>
#include <mutex>
#include <vector>
#include <atomic>
#include <chrono>
#include <thread>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <unordered_map>
#include <mavconn/mavlink_dialect.h>

#pragma once

namespace mavconn {

#define MAX_BUFFER_SIZE 1024

struct Header {
	char marker[3];
	uint8_t topic_ID;
	uint8_t seq;
	uint8_t payload_len_h;
	uint8_t payload_len_l;
	uint8_t crc_h;
	uint8_t crc_l;
};

class CDRConn {
public:
	using ReceivedCb = std::function<void (const uint8_t topic_id, const uint8_t len, const uint8_t* buffer)>;
	void parse_buffer(uint8_t start, uint8_t *buf, const size_t bufsize, size_t bytes_received);
	CDRConn();
	ReceivedCb message_received_cb;
private:
	uint16_t crc16_byte(uint16_t crc, const uint8_t data);
	uint16_t crc16(uint8_t const *buffer, size_t len);
};
}
