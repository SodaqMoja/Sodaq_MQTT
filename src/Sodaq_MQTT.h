/*!
 * \file Sodaq_MQTT.h
 *
 * Copyright (c) 2015 Kees Bakker.  All rights reserved.
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

#ifndef SODAQ_MQTT_H_
#define SODAQ_MQTT_H_

#include <stddef.h>
#include <stdint.h>

#include <Sodaq_MQTT_Interface.h>

/*!
 * \brief The maximum length of a packet
 */
#define MQTT_MAX_PACKET_LENGTH  100
#define MQTT_DEFAULT_KEEP_ALIVE  60

class MQTTPacketInfo;
class MQTT
{
public:
    MQTT();
    void setServer(const char * server, uint16_t port = 1883);
    void setAuth(const char * name, const char * pw);
    void setClientId(const char * id);
    void setTransport(Sodaq_MQTT_Interface * transport);
    void setKeepAlive(uint16_t x) { _keepAlive = x; }

    bool publish(const char * topic, const uint8_t * msg, size_t msg_len, uint8_t qos = 0, uint8_t retain = 1);
    bool publish(const char * topic, const char * msg, uint8_t qos = 0, uint8_t retain = 1);
    bool subscribe(const char * topic, uint8_t qos = 0);
    bool ping();
    void setPublishHandler(void (*handler)(const char *topic, const uint8_t *msg, size_t msg_length));
    void setPacketHandler(void (*handler)(uint8_t *pckt, size_t len));
    bool loop();
    bool availablePacket();
    bool open();
    void close(bool switchOff=true);
    bool isConnected();
    void setStateClosed();

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream &stream) { _diagStream = &stream; }
    void setDiag(Stream *stream) { _diagStream = stream; }

private:
    bool connect();
    bool disconnect();
    size_t assemblePublishPacket(uint8_t * pckt, size_t size,
            const char * topic, const uint8_t * msg, size_t msg_len, uint8_t qos = 0, uint8_t retain = 1);
    size_t assembleSubscribePacket(uint8_t * pckt, size_t size,
            const char * topic, uint8_t qos = 0);
    size_t assembleConnectPacket(uint8_t * pckt, size_t size, uint16_t keepAlive);
    //size_t assembleDisconnectPacket(uint8_t * pckt, size_t size);
    size_t assemblePingreqPacket(uint8_t * pckt, size_t size);
    bool dissectPublishPacket(const uint8_t * pckt, size_t len, MQTTPacketInfo &pckt_info);

    void newPacketIdentifier();

    uint32_t getRemainingLength(const uint8_t *buf, size_t & nrBytes);
    uint16_t get_uint16_be(const uint8_t *buf);

    enum ControlPacketType_e {
        CPT_CONNECT = 1,
        CPT_CONNACK = 2,
        CPT_PUBLISH = 3,
        CPT_PUBACK = 4,
        CPT_PUBREC = 5,
        CPT_PUBREL = 6,
        CPT_PUBCOMP = 7,
	CPT_SUBSCRIBE = 8,
        CPT_SUBACK = 9,
        CPT_UNSUBSCRIBE = 10,
        CPT_UNSUBACK = 11,
        CPT_PINGREQ = 12,
        CPT_PINGRESP = 13,
        CPT_DISCONNECT = 14,
    };
    enum State_e {
        ST_UNKNOWN,
        ST_TCP_OPEN,
        ST_MQTT_CONNECTED,
        ST_MQTT_DISCONNECTED,
        ST_TCP_CLOSED,
    };
    enum State_e _state;
    Sodaq_MQTT_Interface * _transport;
    const char * _server;
    uint16_t _port;
    char * _name;
    char * _password;
    char * _clientId;
    uint16_t _packetIdentifier;
    void (*_publishHandler)(const char *topic, const uint8_t *msg, size_t msg_length);
    void (*_packetHandler)(uint8_t *pckt, size_t len);
    uint16_t _keepAlive;

    // The (optional) stream to show debug information.
    Stream* _diagStream;

    void diagDumpBuffer(const uint8_t * buf, size_t len);
};

/*!
 * \brief This is the default (the only?) instance of the MQTT class
 */
extern MQTT mqtt;

#endif /* SODAQ_MQTT_H_ */
