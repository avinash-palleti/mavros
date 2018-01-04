#include <mavconn/cdrconn.h>

namespace mavconn {


void CDRConn::parse_buffer(uint8_t start, uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
	uint8_t topic_id;
	uint16_t len;
	uint8_t buffer[MAX_BUFFER_SIZE];

	assert(bufsize >= bytes_received);
	struct Header *header = (struct Header *)&buf[start];
	len = ((uint16_t)header->payload_len_h << 8) | header->payload_len_l;
	if (start + len + sizeof(struct Header) > bufsize) {
		return ;    // we don't have a complete msg yet
	}


	// buffer should be big enough to hold a rtps packet
	if (len > MAX_BUFFER_SIZE) {
		return ;
	}

	// Found a whole message, send it and remove from local _buf
	topic_id = header->topic_ID;
	cdr_message_t cdr_message(topic_id, len, buf + start + sizeof(struct Header));
	uint16_t crc = cdr_message.crc16();
	uint16_t read_crc = ((uint16_t)header->crc_h << 8) | header->crc_l;
	memmove(buf + start, buf + start + sizeof(struct Header) + len,
			sizeof(buf) - start - sizeof(struct Header) - len);
	if (crc == read_crc)
		if (message_received_cb) {
			message_received_cb(&cdr_message);
		}
}

CDRConn::CDRConn()
{
// Default constructor
// Initialize map between class names and topic_id
uorbMap.insert(std::make_pair(typeid(home_position_).name(), 33));

}

}
