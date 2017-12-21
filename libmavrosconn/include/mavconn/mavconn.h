
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

enum class Framing : uint8_t {
	incomplete = mavlink::MAVLINK_FRAMING_INCOMPLETE,
	ok = mavlink::MAVLINK_FRAMING_OK,
	bad_crc = mavlink::MAVLINK_FRAMING_BAD_CRC,
	bad_signature = mavlink::MAVLINK_FRAMING_BAD_SIGNATURE,
};

class MAVConn
{
public:
	using ReceivedCb =
		std::function<void (const mavlink::mavlink_message_t *message, const Framing framing)>;
	void parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize, size_t bytes_received);
	MAVConn();
	ReceivedCb message_received_cb;
	mavlink::mavlink_status_t get_status();
	inline mavlink::mavlink_status_t *get_status_p()
	{
		return &m_status;
	}

	inline mavlink::mavlink_message_t *get_buffer_p()
	{
		return &m_buffer;
	}

private:
	mavlink::mavlink_status_t m_status;
	mavlink::mavlink_message_t m_buffer;
};
}
