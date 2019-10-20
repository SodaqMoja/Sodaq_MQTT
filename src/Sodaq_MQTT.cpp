/*!
 * \file Sodaq_MQTT.cpp
 *
 * Copyright (c) 2015-2016 Kees Bakker.  All rights reserved.
 *
 * This file is part of Sodaq_MQTT.
 *
 * Sodaq_MQTT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or(at your option) any later version.
 *
 * Sodaq_MQTT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Sodaq_MQTT.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * The Remaining Length is encoded using a variable length encoding scheme which uses a single byte for
 * values up to 127. Larger values are handled as follows. The least significant seven bits of each byte
 * encode the data, and the most significant bit is used to indicate that there are following bytes in the
 * representation. Thus each byte encodes 128 values and a "continuation bit". The maximum number of
 * bytes in the Remaining Length field is four.
 */

#include <Arduino.h>
#include <stdint.h>

#include "Sodaq_MQTT.h"

#define DEBUG
#define DEBUG_PREFIX String("[MQTT]")

#ifdef DEBUG
#define debugPrintLn(...)   do { if (this->_diagStream) this->_diagStream->println(__VA_ARGS__); } while(0)
#define debugPrint(...)     do { if (this->_diagStream) this->_diagStream->print(__VA_ARGS__); } while (0)
#define debugDump(buf, len) do { this->diagDumpBuffer(buf, len); } while (0)
#else
#define debugPrintLn(...)
#define debugPrint(...)
#define debugDump(buf, len)
#endif

class MQTTPacketInfo
{
public:
    size_t _pckt_length;

    char *_topic;
    size_t _topic_size;
    size_t _topic_length;

    uint8_t *_msg;
    size_t _msg_size;
    size_t _msg_length;
    size_t _msg_truncated_length;

    bool _dup;
    uint8_t _qos;
    bool _retain;

    uint16_t _msg_id;
};

MQTT::MQTT()
{
    _state = ST_UNKNOWN;
    _transport = 0;
    _server = 0;
    _port = 1883;
    _name = 0;
    _password = 0;
    _clientId = 0;
    _packetIdentifier = 0;
    _publishHandler = 0;
    _packetHandler = 0;
    _keepAlive = MQTT_DEFAULT_KEEP_ALIVE;
    _diagStream = 0;
}

/*!
 * \brief Set server and port
 */
void MQTT::setServer(const char * server, uint16_t port)
{
    _server = server;
    _port = port;
}

/*!
 * \brief Set authorization name and password
 */
void MQTT::setAuth(const char * name, const char * pw)
{
    if (name && *name) {
        if (!_name || strcmp(_name, name) != 0) {
            size_t len = strlen(name);
            _name = static_cast<char*>(realloc(_name, len + 1));
            strcpy(_name, name);
        }
    } else {
        free(_name);
        _name = 0;
    }

    if (pw && *pw) {
        if (!_password || strcmp(_password, pw) != 0) {
            size_t len = strlen(pw);
            _password = static_cast<char*>(realloc(_password, len + 1));
            strcpy(_password, pw);
        }
    } else {
        free(_password);
        _password = 0;
    }
}

/*!
 * \brief Set the client ID for MQTT
 *
 * WARNING: This does a malloc
 */
void MQTT::setClientId(const char * id)
{
    if (id && *id) {
        if (!_clientId || strcmp(_clientId, id) != 0) {
            size_t len = strlen(id);
            _clientId = static_cast<char*>(realloc(_clientId, len + 1));
            strcpy(_clientId, id);
        }
    } else {
        free(_clientId);
        _clientId = 0;
    }
}

static void setMQTTStateClosed(void)
{
    mqtt.setStateClosed();
}

/*!
 * \brief Set the transport (MQTT Interface) for MQTT
 */
void MQTT::setTransport(Sodaq_MQTT_Interface * transport)
{
    _transport = transport;
    if (_transport) {
        // We can only handle one MQTT instance.
        // Is this the one MQTT instance?
        if (this == &mqtt) {
            _transport->setMQTTClosedHandler(setMQTTStateClosed);
        }
    }
}

/*!
 * \brief Publish a message
 * \param topic The topic of the publish
 * \param msg The (binary) message to publish
 * \param msg_len The length of the (binary) message
 *
 * Create a PUBLISH packet and send it to the MQTT server
 *
 * \returns false if sending the message failed somehow
 */
bool MQTT::publish(const char * topic, const uint8_t * msg, size_t msg_len, uint8_t qos, uint8_t retain)
{
    debugPrintLn(DEBUG_PREFIX + "PUBLISH topic: " + topic);
    debugPrintLn(DEBUG_PREFIX + "PUBLISH msg: " + (const char *)msg);
    debugDump(msg, msg_len);
    bool retval = false;

    if (_transport == 0) {
        goto ending;
    }

    if (_state != ST_MQTT_CONNECTED) {
        if (!connect()) {
            goto ending;
        }
    }

    newPacketIdentifier();

    uint8_t pckt[MQTT_MAX_PACKET_LENGTH];
    size_t pckt_len;
    // Assemble the PUBLISH packet
    pckt_len = assemblePublishPacket(pckt, sizeof(pckt), topic, msg, msg_len, qos, retain);
    if (pckt_len == 0 || !_transport->sendMQTTPacket(pckt, pckt_len)) {
        goto ending;
    }

    if (qos == 0) {
        // Nothing to be received
    } else if (qos == 1) {
        // Handle incoming PUBACK
        // Expecting PUBACK 4? 02 ?? ??
        size_t pckt_size;
        uint8_t mqtt_puback[4];
        uint16_t pckt_id;
        pckt_size = _transport->receiveMQTTPacket(mqtt_puback, sizeof(mqtt_puback));
        if (pckt_size == 0) {
            debugPrintLn(DEBUG_PREFIX + " timed out");
            goto ending;
        }
        if (pckt_size != sizeof(mqtt_puback)) {
            debugPrintLn(DEBUG_PREFIX + " wrong pckt_size " + pckt_size);
            goto ending;
        }
        if (mqtt_puback[0] != (CPT_PUBACK << 4)) {
            debugPrintLn(DEBUG_PREFIX + " not PUBACK");
            goto ending;
        }
        if (mqtt_puback[1] != 2) {
            debugPrintLn(DEBUG_PREFIX + " not correct length");
            goto ending;
        }
        pckt_id = ((uint16_t)mqtt_puback[2] << 8) | mqtt_puback[3];
        if (pckt_id != _packetIdentifier) {
            debugPrintLn(DEBUG_PREFIX + " wrong packet identifier");
            goto ending;
        }
    } else if (qos == 2) {
        // Handle incoming PUBREC
        // TODO
    } else {
        // Shouldn't happen
    }
    retval = true;

ending:
    return retval;
}

/*!
 * \brief Publish a message
 * \param topic The topic of the publish
 * \param msg The (ASCII) message to publish
 *
 * Create a PUBLISH packet and send it to the MQTT server
 *
 * \returns false if sending the message failed somehow
 */
bool MQTT::publish(const char * topic, const char * msg, uint8_t qos, uint8_t retain)
{
    return publish(topic, (const uint8_t *)msg, strlen(msg), qos, retain);
}

/*!
 * \brief Subscribe
 * \param topic The topic of the publish
 *
 * Create a SUBSCRIBE packet and send it to the MQTT server
 *
 * \returns false if sending the message failed somehow
 *
 * This function is just sending the SUBSCRIBE command. It does
 * not receive any message.  Receiving subscribe packets must be done
 * by polling with the function getSubscribeMessage.
 */
bool MQTT::subscribe(const char * topic, uint8_t qos)
{
    debugPrintLn(DEBUG_PREFIX + "SUBSCRIBE topic: " + topic);
    bool retval = false;

    if (_transport == 0) {
        goto ending;
    }

    if (_state != ST_MQTT_CONNECTED) {
        if (!connect()) {
            goto ending;
        }
    }

    newPacketIdentifier();

    uint8_t pckt[MQTT_MAX_PACKET_LENGTH];
    size_t pckt_len;
    // Assemble the SUBSCRIBE packet
    pckt_len = assembleSubscribePacket(pckt, sizeof(pckt), topic, qos);
    if (pckt_len == 0 || !_transport->sendMQTTPacket(pckt, pckt_len)) {
        debugPrintLn(DEBUG_PREFIX + " failed to send SUBSCRIBE");
        goto ending;
    }

    // Receive the SUBACK packet
    // Expecting SUBACK 90 03 00 01 00
    size_t pckt_size;
    uint8_t mqtt_suback[5];
    uint16_t pckt_id;
    //uint8_t suback_return_code;
    pckt_size = _transport->receiveMQTTPacket(mqtt_suback, sizeof(mqtt_suback));
    if (pckt_size == 0) {
        debugPrintLn(DEBUG_PREFIX + " timed out");
        goto ending;
    }
    if (pckt_size != sizeof(mqtt_suback)) {
        debugPrintLn(DEBUG_PREFIX + " wrong pckt_size " + pckt_size);
        goto ending;
    }
    if (mqtt_suback[0] != (CPT_SUBACK << 4)) {
        debugPrintLn(DEBUG_PREFIX + " not SUBACK");
        goto ending;
    }
    if (mqtt_suback[1] != 3) {
        debugPrintLn(DEBUG_PREFIX + " not correct length");
        goto ending;
    }
    pckt_id = ((uint16_t)mqtt_suback[2] << 8) | mqtt_suback[3];
    if (pckt_id != _packetIdentifier) {
        debugPrintLn(DEBUG_PREFIX + " wrong packet identifier");
        goto ending;
    }
    //suback_return_code = mqtt_suback[4];
    // TODO Decide what we want to do with this

    retval = true;

ending:
    return retval;
}

bool MQTT::ping()
{
    debugPrintLn(DEBUG_PREFIX + "PINGREQ");
    bool retval = false;

    if (_transport == 0) {
        goto ending;
    }

    if (_state != ST_MQTT_CONNECTED) {
        if (!connect()) {
            goto ending;
        }
    }

    newPacketIdentifier();

    uint8_t pckt[MQTT_MAX_PACKET_LENGTH];
    size_t pckt_len;
    // Assemble the SUBSCRIBE packet
    pckt_len = assemblePingreqPacket(pckt, sizeof(pckt));
    if (pckt_len == 0 || !_transport->sendMQTTPacket(pckt, pckt_len)) {
        debugPrintLn(DEBUG_PREFIX + " failed to send PINGREQ");
        goto ending;
    }

    // Receive the PINGRESP packet
    // Expecting PINGRESP D0 00
    size_t pckt_size;
    uint8_t reply_pckt[2];
    //uint8_t suback_return_code;
    pckt_size = _transport->receiveMQTTPacket(reply_pckt, sizeof(reply_pckt));
    if (pckt_size == 0) {
        debugPrintLn(DEBUG_PREFIX + " timed out");
        goto ending;
    }
    if (pckt_size != sizeof(reply_pckt)) {
        debugPrintLn(DEBUG_PREFIX + " wrong pckt_size " + pckt_size);
        goto ending;
    }
    if (reply_pckt[0] != (CPT_PINGRESP << 4)) {
        debugPrintLn(DEBUG_PREFIX + " not PINGRESP");
        goto ending;
    }
    if (reply_pckt[1] != 0) {
        debugPrintLn(DEBUG_PREFIX + " not correct length");
        goto ending;
    }

    retval = true;

ending:
    return retval;
}

/*!
 * \brief do stuff
 *
 * This function is used to do the following:
 *  - read incoming packets and call handlers (if there is one)
 */
bool MQTT::loop()
{
    // Is there a packet?
    bool status;
    size_t pckt_size;
    size_t pckt_ix;
    pckt_size = _transport->availableMQTTPacket();
    if (pckt_size > 0) {
        uint8_t mqtt_packets[256];
        char topic[128];
        uint8_t msg[128];
        MQTTPacketInfo pckt_info;

        pckt_size = _transport->receiveMQTTPacket(mqtt_packets, sizeof(mqtt_packets));
        if (pckt_size > 0) {
            // TODO

            debugPrintLn(DEBUG_PREFIX + " received packet:");
            debugDump(mqtt_packets, pckt_size);

            // Notice that there can be multiple MQTT packet
            pckt_ix = 0;
            while (pckt_ix < pckt_size) {
                switch ((mqtt_packets[pckt_ix] >> 4) & 0xF) {
                case CPT_PUBLISH:

                    memset(&pckt_info, 0, sizeof(pckt_info));
                    pckt_info._topic = topic;
                    pckt_info._topic_size = sizeof(topic);
                    pckt_info._msg = msg;
                    pckt_info._msg_size = sizeof(msg);

                    status = dissectPublishPacket(&mqtt_packets[pckt_ix], pckt_size - pckt_ix, pckt_info);
                    if (status) {
                        if (pckt_info._qos == 0) {
                            // Nothing else to do
                        } else if (pckt_info._qos == 1) {
                            // TODO
                            // Send PUBACK
                        } else if (pckt_info._qos == 2) {
                            // TODO
                            // Send PUBREC
                        } else {
                            // Shouldn't happen
                        }

                        if (_publishHandler) {
                            _publishHandler(topic, msg, pckt_info._msg_truncated_length);
                        }
                        pckt_ix += pckt_info._pckt_length;
                    }
                    else {
                        // TODO
                        // We don't know if we can trust the computed length
                        // For now skip the rest
                        pckt_ix = pckt_size;
                    }
                    break;
                default:
                    if (_packetHandler) {
                        _packetHandler(&mqtt_packets[pckt_ix], pckt_size - pckt_ix);
                    }
                    // TODO
                    // We don't know the packet size. There can be multiple
                    // For now skip the rest
                    pckt_ix = pckt_size;
                    break;
                }
            }
        }
        return true;
    }

    return false;
}

/*!
 * Is there a packet available
 */
bool MQTT::availablePacket()
{
    return _transport->availableMQTTPacket() > 0;
}

/*!
 * \brief Close the connection
 */
void MQTT::close(bool switchOff)
{
    if (_state == ST_MQTT_CONNECTED) {
        disconnect();
    }

    _transport->closeMQTT(switchOff);
    _state = ST_TCP_CLOSED;
}

/*!
 * \brief Is the connection active
 */
bool MQTT::isConnected()
{
    return _transport->isAliveMQTT();
}

/*!
 * \brief Set state TCP closed
 *
 * Use this function when the modem issues an URC with "connection closed"
 */
void MQTT::setStateClosed()
{
    _state = ST_TCP_CLOSED;
}

void MQTT::setPublishHandler(void (*handler)(const char *topic, const uint8_t *msg, size_t msg_length))
{
    _publishHandler = handler;
}

void MQTT::setPacketHandler(void (*handler)(uint8_t *pckt, size_t len))
{
    _packetHandler = handler;
}

/*!
 * \brief Open the MQTT connection
 */
bool MQTT::open()
{
    if (_state != ST_TCP_OPEN) {
        if (_transport->openMQTT(_server, _port)) {
            _state = ST_TCP_OPEN;
        }
    }
    return _state == ST_TCP_OPEN;
}

/*!
 * \brief Connect to the MQTT server
 */
bool MQTT::connect()
{
    debugPrintLn(DEBUG_PREFIX + "CONNECT");
    bool retval = false;

    if (!open()) {
        goto ending;
    }

    // Assemble a CONNECT packet
    uint8_t pckt[MQTT_MAX_PACKET_LENGTH];
    size_t pckt_len;
    pckt_len = assembleConnectPacket(pckt, sizeof(pckt), _keepAlive);
    if (pckt_len == 0 || !_transport->sendMQTTPacket(pckt, pckt_len)) {
        goto ending;
    }

    // Receive the CONNACK packet
    // Expecting CONNACK 20 02 00 00
    size_t pckt_size;
    uint8_t mqtt_connack[4];
    pckt_size = _transport->receiveMQTTPacket(mqtt_connack, sizeof(mqtt_connack));
    if (pckt_size == 0) {
        debugPrintLn(DEBUG_PREFIX + " timed out");
        goto ending;
    }
    if (pckt_size != sizeof(mqtt_connack)) {
        debugPrintLn(DEBUG_PREFIX + " wrong pckt_size " + pckt_size);
        goto ending;
    }
    if (pckt_size != sizeof(mqtt_connack)) {
        goto ending;
    }
    if (mqtt_connack[0] != (CPT_CONNACK << 4)) {
        debugPrintLn(DEBUG_PREFIX + " not CONNACK, but " + (mqtt_connack[0] >> 4));
        goto ending;
    }
    // Return code
    if (mqtt_connack[3] != 0) {
        debugPrintLn(DEBUG_PREFIX + " connection not accepted, return code " + mqtt_connack[3]);
        goto ending;
    }

    // All went well
    _state = ST_MQTT_CONNECTED;
    retval = true;

ending:
    return retval;
}

/*!
 * \brief Disconnect from the MQTT server
 *
 * This is done by sending a DISCONNECT packet
 */
bool MQTT::disconnect()
{
    debugPrintLn(DEBUG_PREFIX + "DISCONNECT");
    bool retval = false;
    uint8_t pckt[2];
    pckt[0] = (CPT_DISCONNECT << 4);
    pckt[1] = 0;
    if (!_transport->sendMQTTPacket(pckt, 2)) {
        goto ending;
    }
    retval = true;
ending:
    return retval;
}

/*!
 * \brief Assemble a CONNECT packet
 *
 * \returns The size of the assembled packet.
 */
size_t MQTT::assembleConnectPacket(uint8_t * pckt, size_t size, uint16_t keepAlive)
{
    // Assume pckt is not NULL
    // If name is not NULL then pw should be not NULL either.
    uint8_t * ptr = pckt;

    const char * protocol_name = "MQTT";
    const int protocol_level = 4;

    size_t len;
    size_t pckt_len = 2 + strlen(protocol_name)
              + 4
              + 2 + strlen(_clientId)
              + 2 + strlen(_name)
              + 2 + strlen(_password);
    if (size < (pckt_len + 2)) {
        // Oops. It does not fit. Truncate and hope for the best.
        pckt_len = size - 2;
    }

    *ptr++ = (CPT_CONNECT << 4) | 0;
    // Assume length smaller than 128, or else we need multi byte length
    *ptr++ = pckt_len;

    // Variable header:
    //   Protocol Name,
    len = strlen(protocol_name);
    *ptr++ = highByte(len);
    *ptr++ = lowByte(len);
    memcpy(ptr, protocol_name, len);
    ptr += len;

    //   Protocol Level,
    *ptr++ = protocol_level;
    //   Connect Flags,
    uint8_t flags = 0;
    if (_name != 0 && _password != 0) {
        flags |= (1 << 7) | (1 << 6);
    }
    flags |= (1 << 1);            // clean session
    *ptr++ = flags;

    *ptr++ = keepAlive >> 8;
    *ptr++ = keepAlive & 0xFF;

    len = strlen(_clientId);
    *ptr++ = highByte(len);
    *ptr++ = lowByte(len);
    memcpy(ptr, _clientId, len);
    ptr += len;

    len = strlen(_name);
    *ptr++ = highByte(len);
    *ptr++ = lowByte(len);
    memcpy(ptr, _name, len);
    ptr += len;

    len = strlen(_password);
    *ptr++ = highByte(len);
    *ptr++ = lowByte(len);
    memcpy(ptr, _password, len);
    ptr += len;

    debugPrintLn(DEBUG_PREFIX + "CONNECT packet:");
    debugDump(pckt, pckt_len + 2);

    return pckt_len + 2;
}

/*!
 * \brief Assemble a PUBLISH packet
 *
 * \returns The size of the assembled packet.
 */
size_t MQTT::assemblePublishPacket(uint8_t * pckt, size_t size,
    const char * topic, const uint8_t * msg, size_t msg_len, uint8_t qos, uint8_t retain)
{
    // Assume pckt is not NULL
    uint8_t * ptr = pckt;

    // First compute the "remaining length"
    size_t topic_length = strlen(topic);
    size_t remaining = 0;
    remaining += 2 + topic_length;
    if (qos == 1 || qos == 2) {
        remaining += 2;
    }
    remaining += msg_len;
    if (remaining > 127) {
        // TODO We only support max 127 bytes remaining length
        return 0;
    }
    if ((remaining + 2) > size) {
        // Oops. It does not fit.
        return 0;
    }

    // Header
    const uint8_t dup = 0;
    *ptr++ = (CPT_PUBLISH << 4) | ((dup & 0x01) << 3) | ((qos & 0x03) << 1) | ((retain & 0x01) << 0);
    // Assume length smaller than 128, or else we need multi byte length
    *ptr++ = remaining;

    // Add Topic. 2 byte length of topic (MSB, LSB) followed by topic
    *ptr++ = highByte(topic_length);
    *ptr++ = lowByte(topic_length);
    memcpy(ptr, topic, topic_length);
    ptr += topic_length;

    // Add (optional) Packet Identifier
    if (qos == 1 || qos == 2) {
        // Packet Identifier only if QoS 1 or 2
        *ptr++ = (_packetIdentifier >> 8) & 0xFF;
        *ptr++ = _packetIdentifier & 0xFF;
    }

    // Add Payload = topic message
    memcpy(ptr, msg, msg_len);

    debugPrintLn(DEBUG_PREFIX + "PUBLISH packet:");
    debugDump(pckt, remaining + 2);

    return remaining + 2;
}

/*!
 * \brief Assemble a PINGREQ packet
 *
 * \returns The size of the assembled packet.
 */
size_t MQTT::assemblePingreqPacket(uint8_t * pckt, size_t size)
{
    // Assume pckt is not NULL
    uint8_t * ptr = pckt;

    size_t remaining = 0;

    // Header
    *ptr++ = (CPT_PINGREQ << 4);
    *ptr++ = remaining;

    debugPrintLn(DEBUG_PREFIX + "PINGREQ packet:");
    debugDump(pckt, remaining + 2);

    return remaining + 2;
}

/*!
 * \brief Assemble a SUBSCRIBE packet
 * \param pckt The buffer to store the assembled packet
 * \param topic The size of the pckt buffer
 * \param topic The topic filter of the subscribe
 * \param qos The QoS of the topic
 *
 * \returns The size of the assembled packet.
 *
 * The SUBSCRIBE packet:
 *   1 byte length
 *   2 bytes Packet Identifier
 *   One or more Topic Filers, each:
 *     2 bytes length
 *     N bytes topic
 *     1 byte QoS (only bits 0, 1)
 * In this function we only have one topic filter.
 */
size_t MQTT::assembleSubscribePacket(uint8_t * pckt, size_t size,
    const char * topic, uint8_t qos)
{
    // Assume pckt is not NULL
    uint8_t * ptr = pckt;

    const int topic_extra = 3;          // 2 bytes length, 1 byte QoS
    int max_topic_length = size - (2 + topic_extra);
    int topic_length = strlen(topic);
    if (topic_length > max_topic_length) {
        // Oops. It does not fit. Truncate and hope for the best.
        topic_length = max_topic_length;
    }

    size_t pckt_len = 2 + topic_length + topic_extra;
    *ptr++ = (CPT_SUBSCRIBE << 4) | (2 << 0);         // reserved field must be 0010
    // Assume length smaller than 128, or else we need multi byte length, see note about
    // Remaining Length above
    *ptr++ = pckt_len;

    *ptr++ = (_packetIdentifier >> 8) & 0xFF;
    *ptr++ = _packetIdentifier & 0xFF;

    // 2 byte length of topic (MSB, LSB) followed by topic
    *ptr++ = highByte(topic_length);
    *ptr++ = lowByte(topic_length);
    memcpy(ptr, topic, topic_length);
    ptr += topic_length;
    *ptr++ = qos & 0x3;

    debugPrintLn(DEBUG_PREFIX + "SUBSCRIBE packet:");
    debugDump(pckt, pckt_len + 2);

    return pckt_len + 2;
}

/*
 * \brief Dissect a PUBLISH packet
 * \param[in] pckt The buffer to store the assembled packet
 * \param[in] len The length of the pckt buffer
 * \param[out] topic The buffer to store the topic
 * \param[in] topic_size The size of buffer to store the topic
 * \param[out] msg The buffer to store the message
 * \param[in] msg_size The size of buffer to store the message
 * \param[out] msg_length The length of the message
 *
 * The PUBLISH packet contains the following:
 *   - fixed header containg:
 *   -- msg type | DUP | QoS | RETAIN
 *   -- remaining length (1..4 bytes)
 *   - variable header containing:
 *   -- topic
 *   -- message ID (only if QoS level 1 or 2)
 *   - payload (can be 0 bytes or more)
 *
 * The caller is responsible for the receiving message (payload). For this
 * function the payload is binary, not text. If the caller knows that the
 * message is text it should pass a size which is one smaller that the allocated
 * space, and that one byte extra must be set to zero before using the string.
 *
 * The \a msg_length is the truncated length of the message if the actual message
 * was more that what fits in \a msg.
 */
bool MQTT::dissectPublishPacket(const uint8_t * pckt, size_t len, MQTTPacketInfo &pckt_info)
{
    const uint8_t *ptr;

    debugPrintLn(DEBUG_PREFIX + "  dissectPublish");
    ptr = pckt;

    // uint8_t msg_type = (*ptr >> 4) & 0xF;
    pckt_info._dup = ((*ptr >> 3) & 0x1) ? true : false;
    pckt_info._qos = (*ptr >> 1) & 0x3;
    pckt_info._retain = ((*ptr >> 0) & 0x1) ? true : false;

    ptr++;

    // Get the "remaining length"
    size_t nrBytesRL = 0;
    uint32_t remaining = getRemainingLength(ptr, nrBytesRL);
    ptr += nrBytesRL;
    debugPrintLn(DEBUG_PREFIX + "    remaining=" + remaining);

    pckt_info._pckt_length = 1 + nrBytesRL + remaining;
    debugPrintLn(DEBUG_PREFIX + "    total size=" + pckt_info._pckt_length);

    pckt_info._topic_length = get_uint16_be(ptr);
    ptr += 2;
    remaining -= pckt_info._topic_length + 2;

    memset(pckt_info._topic, 0, pckt_info._topic_size);
    size_t my_topic_length = pckt_info._topic_length;
    if (my_topic_length > (pckt_info._topic_size - 1)) {
        // Maximize so that it fits in the given buffer, with NUL
        my_topic_length = pckt_info._topic_size - 1;
    }
    memcpy(pckt_info._topic, ptr, my_topic_length);
    ptr += pckt_info._topic_length;
    debugPrintLn(DEBUG_PREFIX + "    topic length=" + pckt_info._topic_length);
    debugPrintLn(DEBUG_PREFIX + "    topic=" + pckt_info._topic);

    pckt_info._msg_id = 0;
    if (pckt_info._qos == 1 || pckt_info._qos == 2) {
        pckt_info._msg_id = get_uint16_be(ptr);
        ptr += 2;
        debugPrintLn(DEBUG_PREFIX + "    msg ID=" + pckt_info._msg_id);
    }

    pckt_info._msg_length = remaining;
    memset(pckt_info._msg, 0, pckt_info._msg_size);
    pckt_info._msg_truncated_length = pckt_info._msg_length;
    if (pckt_info._msg_truncated_length > pckt_info._msg_size) {
        pckt_info._msg_truncated_length = pckt_info._msg_size;
    }
    memcpy(pckt_info._msg, ptr, pckt_info._msg_truncated_length);
    debugPrintLn(DEBUG_PREFIX + "    msg length=" + pckt_info._msg_length);
    debugPrintLn(DEBUG_PREFIX + "    truncated msg length=" + pckt_info._msg_truncated_length);

    return true;
}

/*
 * Set new Packet Identifier
 */
void MQTT::newPacketIdentifier()
{
    ++_packetIdentifier;
    if (_packetIdentifier == 0) {
        // Cannot be zero
        ++_packetIdentifier;
    }
}

uint32_t MQTT::getRemainingLength(const uint8_t *buf, size_t & nrBytes)
{
#if 0
    // From the MQTT document
    uint32_t value;
    uint32_t multiplier;
    uint8_t digit;
    nrBytes = 0;

    multiplier = 1;
    value = 0;
    do {
        digit = *buf++;
        nrBytes++;
        value += (digit & 0x7F) * multiplier;
        multiplier *= 128;
    } while ((digit & 0x80) != 0);
#else
    uint32_t value;
    int shift;
    uint8_t digit;
    nrBytes = 0;

    shift = 0;
    value = 0;
    do {
        digit = *buf++;
        nrBytes++;
        value += (digit & 0x7F) << shift;
        shift += 7;
    } while ((digit & 0x80) != 0);
#endif
    return value;
}

uint16_t MQTT::get_uint16_be(const uint8_t * buf)
{
    return (uint16_t)(buf[0] << 8) | buf[1];
}

/*
 * Dump a buffer for diagnostic purposes
 */
void MQTT::diagDumpBuffer(const uint8_t * buf, size_t len)
{
    if (this->_diagStream == 0) {
        return;
    }
    for (size_t ix = 0; ix < len; ++ix) {
        if ((ix % 16) == 0) {
            if (ix != 0) {
                debugPrintLn();
            }
        }
        uint8_t val = buf[ix];
        debugPrint((val >> 4), HEX);
        debugPrint(val & 0xF, HEX);
    }
    if (len > 0) {
        debugPrintLn();
    }
}

MQTT mqtt;
