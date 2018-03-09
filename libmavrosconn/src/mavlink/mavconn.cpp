#include <mavconn/mavconn.h>

namespace mavconn {
void MAVConn::parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize,
						   size_t bytes_received)
{
	mavlink::mavlink_status_t status;
	mavlink::mavlink_message_t message;

	assert(bufsize >= bytes_received);

	for (; bytes_received > 0; bytes_received--) {
		auto c = *buf++;

		// based on mavlink_parse_char()
		auto msg_received = static_cast<Framing>(mavlink::mavlink_frame_char_buffer(&m_buffer, &m_status, c,
																					&message, &status));
		if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
			mavlink::_mav_parse_error(&m_status);
			m_status.msg_received = mavlink::MAVLINK_FRAMING_INCOMPLETE;
			m_status.parse_state = mavlink::MAVLINK_PARSE_STATE_IDLE;
			if (c == MAVLINK_STX) {
				m_status.parse_state = mavlink::MAVLINK_PARSE_STATE_GOT_STX;
				m_buffer.len = 0;
				mavlink::mavlink_start_checksum(&m_buffer);
			}
		}

		if (msg_received != Framing::incomplete) {
			if (message_received_cb) {
				message_received_cb(&message, msg_received);
			}
		}
	}
}

mavlink::mavlink_status_t MAVConn::get_status()
{
	return m_status;
}

MAVConn::MAVConn() :
	m_buffer{},
	m_status{}
{
// Default constructor
}

}
