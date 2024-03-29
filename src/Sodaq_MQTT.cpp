/*!
 * \file Sodaq_MQTT.cpp
 *
 * Copyright (c) 2015-2020 Kees Bakker.  All rights reserved.
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

/**
 * \def DEBUG Enable this define to get debug (diag) output
 */
#define DEBUG

/**
 * \def DEBUG_DUMP_PACKETS Enable this define to also enable diag of packets
 */
//#define DEBUG_DUMP_PACKETS

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

static inline bool is_timedout(uint32_t from, uint32_t nr_ms) __attribute__((always_inline));
static inline bool is_timedout(uint32_t from, uint32_t nr_ms) { return (millis() - from) > nr_ms; }

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
    _waiting_for_ack = false;
    _ack_was_ok = false;;
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
#ifdef DEBUG_DUMP_PACKETS
    debugDump(msg, msg_len);
#endif
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

    size_t pckt_len;
    // Assemble the PUBLISH packet
    pckt_len = assemblePublishPacket(_pckt, sizeof(_pckt), topic, msg, msg_len, qos, retain);
    debugPrintLn(String(DEBUG_PREFIX + "pckt_len: ") + pckt_len);
    if (pckt_len == 0 || !_transport->sendMQTTPacket(_pckt, pckt_len)) {
        goto ending;
    }

    if (qos == 0) {
        // Nothing to be received
        retval = true;
    } else if (qos == 1) {
        retval = waitForAck("PUBACK");
    } else if (qos == 2) {
        // Handle incoming PUBREC
        // TODO
    } else {
        // Shouldn't happen
    }

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

/**
 * Handle incoming PUBACK
 *
 * Expecting PUBACK 4? 02 ?? ??
 *
 * This function is being called from loop()
 */
size_t MQTT::handlePUBACK(uint8_t *pckt, size_t len)
{
    const size_t pckt_len = 4;
    debugPrintLn(DEBUG_PREFIX + " got PUBACK");
    _waiting_for_ack = false;
    if (len < pckt_len) {
        debugPrintLn(DEBUG_PREFIX + " wrong pckt_size");
        return 0;
    }
    if (pckt[1] != 2) {
        debugPrintLn(DEBUG_PREFIX + " not correct length");
        return 0;
    }
    uint16_t pckt_id= ((uint16_t)pckt[2] << 8) | pckt[3];
    if (pckt_id != _packetIdentifier) {
        debugPrintLn(DEBUG_PREFIX + " wrong packet identifier");
        return 0;
    }

    _ack_was_ok = true;

    /* Return the length of the packet
     */
    return pckt_len;
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

    size_t pckt_len;
    // Assemble the SUBSCRIBE packet
    pckt_len = assembleSubscribePacket(_pckt, sizeof(_pckt), topic, qos);
    if (pckt_len == 0 || !_transport->sendMQTTPacket(_pckt, pckt_len)) {
        debugPrintLn(DEBUG_PREFIX + " failed to send SUBSCRIBE");
        goto ending;
    }

    retval = waitForAck("SUBACK");

ending:
    return retval;
}

/**
 * Handle incoming SUBACK
 *
 * Expecting SUBACK 90 03 00 01 00
 *
 * This function is being called from loop()
 */
size_t MQTT::handleSUBACK(uint8_t *pckt, size_t len)
{
    const size_t pckt_len = 5;
    debugPrintLn(DEBUG_PREFIX + " got SUBACK");
    _waiting_for_ack = false;
    if (len < pckt_len) {
        debugPrintLn(DEBUG_PREFIX + " wrong pckt_size");
        return 0;
    }
    if (pckt[1] != 3) {
        debugPrintLn(DEBUG_PREFIX + " not correct length");
        return 0;
    }
    uint16_t pckt_id= ((uint16_t)pckt[2] << 8) | pckt[3];
    if (pckt_id != _packetIdentifier) {
        debugPrintLn(DEBUG_PREFIX + " wrong packet identifier");
        return 0;
    }
    //suback_return_code = mqtt_suback[4];
    // TODO Decide what we want to do with this

    _ack_was_ok = true;

    /* Return the length of the packet
     */
    return pckt_len;
}

bool MQTT::sendPUBACK(uint16_t msg_id)
{
    debugPrintLn(DEBUG_PREFIX + "send PUBACK");
    bool retval = false;

    if (_transport == 0) {
        goto ending;
    }

    if (_state != ST_MQTT_CONNECTED) {
        if (!connect()) {
            goto ending;
        }
    }

    uint8_t pckt[MQTT_MAX_PACKET_LENGTH];
    size_t pckt_len;
    // Assemble the PUBACK packet
    pckt_len = assemblePubackPacket(pckt, sizeof(pckt), msg_id);
    if (pckt_len == 0 || !_transport->sendMQTTPacket(pckt, pckt_len)) {
        debugPrintLn(DEBUG_PREFIX + " failed to send PUBACK");
        goto ending;
    }
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

    uint8_t pckt[MQTT_MAX_PACKET_LENGTH];
    size_t pckt_len;
    // Assemble the SUBSCRIBE packet
    pckt_len = assemblePingreqPacket(pckt, sizeof(pckt));
    if (pckt_len == 0 || !_transport->sendMQTTPacket(pckt, pckt_len)) {
        debugPrintLn(DEBUG_PREFIX + " failed to send PINGREQ");
        goto ending;
    }

    retval = waitForAck("PINGRESP");

ending:
    return retval;
}

/**
 * Handle incoming PINGRESP
 *
 * Expecting PINGRESP D0 00
 *
 * This function is being called from loop()
 */
size_t MQTT::handlePINGRESP(uint8_t *pckt, size_t len)
{
    const size_t pckt_len = 2;
    debugPrintLn(DEBUG_PREFIX + " got PINGRESP");
    _waiting_for_ack = false;
    if (len < pckt_len) {
        debugPrintLn(DEBUG_PREFIX + " wrong pckt_size");
        return 0;
    }
    if (pckt[1] != 0) {
        debugPrintLn(DEBUG_PREFIX + " not correct length");
        return 0;
    }

    _ack_was_ok = true;

    /* Return the length of the packet
     */
    return pckt_len;
}

bool MQTT::waitForAck(const String& expect)
{
    debugPrintLn(DEBUG_PREFIX + " expect " + expect);
    /* Handle incoming PINGRESP in the next iteration of loop()
     * Assume the it will fail.
     */
    _waiting_for_ack = true;
    _ack_was_ok = false;
    /* Wait for PUBACK, SUBACK, PINGRESP, etc
     * How can we limit in case of failure?
     */
    uint32_t start_ts = millis();
    while (!is_timedout(start_ts, MQTT_WAIT_FOR_ACK_TIMEOUT) &&
           isConnected() && _waiting_for_ack) {
        loop();
    }
    /* Clear the flag, even if we didn't get the ack
     */
    _waiting_for_ack = false;
    return _ack_was_ok;
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
    size_t pckt_len;
    pckt_size = _transport->availableMQTTPacket();
    if (pckt_size > 0) {
        uint8_t mqtt_packets[256];
        char topic[128];
        uint8_t msg[128];
        MQTTPacketInfo pckt_info;

        pckt_size = _transport->receiveMQTTPacket(mqtt_packets, sizeof(mqtt_packets));
        if (pckt_size > 0) {
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
                            (void)sendPUBACK(pckt_info._msg_id);
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

                case CPT_CONNACK:
                    pckt_len = handleCONNACK(&mqtt_packets[pckt_ix], pckt_size - pckt_ix);
                    if (pckt_len == 0) {
                        /* Something is wrong. Skip the rest
                         */
                        pckt_ix = pckt_size;
                    }
                    else {
                        pckt_ix += pckt_len;
                    }
                    break;

                case CPT_PUBACK:
                    pckt_len = handlePUBACK(&mqtt_packets[pckt_ix], pckt_size - pckt_ix);
                    if (pckt_len == 0) {
                        /* Something is wrong. Skip the rest
                         */
                        pckt_ix = pckt_size;
                    }
                    else {
                        pckt_ix += pckt_len;
                    }
                    break;

                case CPT_SUBACK:
                    pckt_len = handleSUBACK(&mqtt_packets[pckt_ix], pckt_size - pckt_ix);
                    if (pckt_len == 0) {
                        /* Something is wrong. Skip the rest
                         */
                        pckt_ix = pckt_size;
                    }
                    else {
                        pckt_ix += pckt_len;
                    }
                    break;

                case CPT_PINGRESP:
                    pckt_len = handlePINGRESP(&mqtt_packets[pckt_ix], pckt_size - pckt_ix);
                    if (pckt_len == 0) {
                        /* Something is wrong. Skip the rest
                         */
                        pckt_ix = pckt_size;
                    }
                    else {
                        pckt_ix += pckt_len;
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
    size_t pckt_len;
    pckt_len = assembleConnectPacket(_pckt, sizeof(_pckt), _keepAlive);
    if (pckt_len == 0 || !_transport->sendMQTTPacket(_pckt, pckt_len)) {
        goto ending;
    }

    retval = waitForAck("CONNACK");
    if (retval) {
        // All went well
        _state = ST_MQTT_CONNECTED;
    }

ending:
    return retval;
}

/**
 * Handle incoming CONNACK
 *
 * Expecting CONNACK 20 02 00 00
 *
 * This function is being called from loop()
 */
size_t MQTT::handleCONNACK(uint8_t *pckt, size_t len)
{
    const size_t pckt_len = 4;
    debugPrintLn(DEBUG_PREFIX + " got CONNACK");
    _waiting_for_ack = false;
    if (len < pckt_len) {
        debugPrintLn(DEBUG_PREFIX + " wrong pckt_size");
        return 0;
    }
    if (pckt[1] != 2) {
        debugPrintLn(DEBUG_PREFIX + " not correct length");
        return 0;
    }
    // Return code
    if (pckt[3] != 0) {
        debugPrintLn(DEBUG_PREFIX + " connection not accepted, return code " + pckt[3]);
        return 0;
    }

    _ack_was_ok = true;

    /* Return the length of the packet
     */
    return pckt_len;
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
              + 2 + strlen(_clientId);
    if (_name) {
        pckt_len += 2 + strlen(_name);
        if (_password) {
            pckt_len += 2 + strlen(_password);
        }
    }
    // +4 is for the max amount of remaining length bytes
    if (size < (pckt_len + 4)) {
        // Oops. It does not fit. Truncate and hope for the best.
        pckt_len = size - 4;
    }

    *ptr++ = (CPT_CONNECT << 4) | 0;

    size_t varint_len = writeRemainingLength(pckt_len, ptr);

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
    if (_name) {
        flags |= (1 << 7);
        if (_password) {
            flags |= (1 << 6);
        }
    }
    flags |= (1 << 1);            // clean session
    *ptr++ = flags;

    *ptr++ = highByte(keepAlive);
    *ptr++ = lowByte(keepAlive);

    len = strlen(_clientId);
    *ptr++ = highByte(len);
    *ptr++ = lowByte(len);
    memcpy(ptr, _clientId, len);
    ptr += len;

    if (_name) {
        len = strlen(_name);
        *ptr++ = highByte(len);
        *ptr++ = lowByte(len);
        memcpy(ptr, _name, len);
        ptr += len;

        if (_password) {
            len = strlen(_password);
            *ptr++ = highByte(len);
            *ptr++ = lowByte(len);
            memcpy(ptr, _password, len);
            ptr += len;
        }
    }

#ifdef DEBUG_DUMP_PACKETS
    debugPrintLn(DEBUG_PREFIX + "CONNECT packet:");
    debugDump(pckt, pckt_len + varint_len + 1);
#endif

    return pckt_len + varint_len + 1;
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
    uint32_t remaining = 0;

    /* The topic is encoded with a 2 byte length followed by the topic
     */
    remaining += 2 + topic_length;
    if (qos == 1 || qos == 2) {
        // Two bytes for the msg_id
        remaining += 2;
    }
    remaining += msg_len;

    /* Compute how many bytes we need.
     * The +1 here is for the first byte
     * and the +4 is for the remaining field (max 4).
     */
    if ((remaining + 1 + 4) > size) {
        // Oops. It does not fit.
        debugPrintLn(String(DEBUG_PREFIX + "packet too big for buffer: ") + remaining);
        return 0;
    }

    // Header
    const uint8_t dup = 0;
    *ptr++ = (CPT_PUBLISH << 4) | ((dup & 0x01) << 3) | ((qos & 0x03) << 1) | ((retain & 0x01) << 0);

    size_t varint_len = writeRemainingLength(remaining, ptr);

    // Add Topic. 2 byte length of topic (MSB, LSB) followed by topic
    *ptr++ = highByte(topic_length);
    *ptr++ = lowByte(topic_length);
    memcpy(ptr, topic, topic_length);
    ptr += topic_length;

    // Add (optional) Packet Identifier
    if (qos == 1 || qos == 2) {
        // Packet Identifier only if QoS 1 or 2
        *ptr++ = highByte(_packetIdentifier);
        *ptr++ = lowByte(_packetIdentifier);
    }

    // Add Payload = topic message
    memcpy(ptr, msg, msg_len);

#ifdef DEBUG_DUMP_PACKETS
    debugPrintLn(DEBUG_PREFIX + "PUBLISH packet:");
    debugDump(pckt, 1 + varint_len + remaining);
#endif

    return 1 + varint_len + remaining;
}

/*!
 * \brief Assemble a PUBACK packet
 *
 * \returns The size of the assembled packet.
 */
size_t MQTT::assemblePubackPacket(uint8_t * pckt, size_t size, uint16_t msg_id)
{
    // Assume pckt is not NULL
    uint8_t * ptr = pckt;

    size_t remaining = 2;

    // Header (DUP, QoS, and RETAIN are not used)
    *ptr++ = (CPT_PUBACK << 4);
    *ptr++ = remaining;

    *ptr++ = highByte(msg_id);
    *ptr++ = lowByte(msg_id);

#ifdef DEBUG_DUMP_PACKETS
    debugPrintLn(DEBUG_PREFIX + "PUBACK packet:");
    debugDump(pckt, remaining + 2);
#endif

    return remaining + 2;
}

/*!
 * \brief Assemble a PINGREQ packet
 *
 * \returns The size of the assembled packet.
 */
size_t MQTT::assemblePingreqPacket(uint8_t * pckt, size_t size)
{
    (void)size;

    // Assume pckt is not NULL
    uint8_t * ptr = pckt;

    size_t remaining = 0;

    // Header
    *ptr++ = (CPT_PINGREQ << 4);
    *ptr++ = remaining;

#ifdef DEBUG_DUMP_PACKETS
    debugPrintLn(DEBUG_PREFIX + "PINGREQ packet:");
    debugDump(pckt, remaining + 2);
#endif

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

    size_t varint_len = writeRemainingLength(pckt_len, ptr);

    *ptr++ = highByte(_packetIdentifier);
    *ptr++ = lowByte(_packetIdentifier);

    // 2 byte length of topic (MSB, LSB) followed by topic
    *ptr++ = highByte(topic_length);
    *ptr++ = lowByte(topic_length);
    memcpy(ptr, topic, topic_length);
    ptr += topic_length;
    *ptr++ = qos & 0x3;

#ifdef DEBUG_DUMP_PACKETS
    debugPrintLn(DEBUG_PREFIX + "SUBSCRIBE packet:");
    debugDump(pckt, pckt_len + varint_len + 1);
#endif

    return pckt_len + varint_len + 1;
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
    (void)len;

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
        remaining -= 2;
        debugPrintLn(DEBUG_PREFIX + "    QoS=" + pckt_info._qos);
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
 * Write a variable length int to the buffer as defined in MQTT spec v3.1.1
 */
size_t MQTT::writeRemainingLength(size_t length, uint8_t * &ptr)
{
    size_t len = 0;
    // from pseudocode https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Toc398718023
    uint8_t encodedByte;
    do {
        encodedByte = length % 128;
        length = length / 128;
        if (length > 0) {
            encodedByte = encodedByte | 128;
        }
        *ptr++ = encodedByte;
        len++;
    } while (length > 0);

    return len;
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
