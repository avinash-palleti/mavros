#include <mavconn/cdrconn.h>
#include <stdio.h>

std::map<std::string, int> uorbMap;
std::map<int, int> cdrMsgMap;

namespace mavconn {

void CDRConn::parse_buffer(uint8_t start, uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
	assert(bufsize >= bytes_received);
	cdr_frame_buffer(buf,bytes_received, start);
	if(buf_len < sizeof(struct Header))
	{
		//Still header yet to be received
		m_status = CDR_FRAME_STATUS::CDR_FRAME_INCOMPLETE;
		return;
	}
	struct Header *header = (struct Header *)&rxmsg;
	uint16_t len = ((uint16_t)header->payload_len_h << 8) | header->payload_len_l;
	if (len > MAX_BUFFER_SIZE) {
		buf_len = 0; //dropping the previous buffer
		m_status = CDR_FRAME_STATUS::CDR_FRAME_IDLE;
		return ;
	}
	if (len + sizeof(struct Header) > buf_len)
	{
		m_status = CDR_FRAME_STATUS::CDR_FRAME_INCOMPLETE;
		return;
	}
	uint8_t topic_id;
	topic_id = header->topic_ID;
	cdr_message_t cdr_message(topic_id, len, rxmsg + start + sizeof(struct Header));
	uint16_t crc = cdr_message.crc16();
	uint16_t read_crc = ((uint16_t)header->crc_h << 8) | header->crc_l;
	if (crc == read_crc)
		if (message_received_cb) {
			message_received_cb(&cdr_message);
		}
	buf_len = 0;
	m_status = CDR_FRAME_STATUS::CDR_FRAME_IDLE;
}
void CDRConn::cdr_frame_buffer(uint8_t *buffer, const size_t bytes_received, uint8_t start)
{
	memcpy(rxmsg+buf_len, buffer+start, bytes_received-start);
	buf_len+=bytes_received-start;
}

CDRConn::CDR_FRAME_STATUS CDRConn::get_cdr_frame_status() 
{
	return m_status;
}

CDRConn::CDRConn()
{
// Default constructor
uorbMap.insert(std::make_pair(typeid(home_position_).name(), 162));
cdrMsgMap[33] = 162;
}

}
