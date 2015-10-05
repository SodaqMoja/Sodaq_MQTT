/*!
 * \file Sodaq_MQTT.cpp
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

#include <Arduino.h>

#include "Sodaq_MQTT.h"

MQTT::MQTT()
{
  _state = ST_UNKNOWN;
  _transport = 0;
  _server = 0;
  _port = 1883;
  _name = 0;
  _password = 0;
  _clientId = 0;
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
  _name = name;
  _password = pw;
}
/*!
 * \brief Set the client ID for MQTT
 */
void MQTT::setClientId(const char * id)
{
  _clientId = id;
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
bool MQTT::publish(const char * topic, const uint8_t * msg, size_t msg_len)
{
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
  // Assemble the PUBLISH packet
  pckt_len = assemblePublishPacket(pckt, sizeof(pckt), topic, msg, msg_len);
  if (!_transport->sendPacket(pckt, pckt_len)) {
    goto ending;
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
bool MQTT::publish(const char * topic, const char * msg)
{
  return publish(topic, (const uint8_t *)msg, strlen(msg));
}

/*!
 * \brief Close the connection
 */
void MQTT::close()
{
  if (_state == ST_MQTT_CONNECTED) {
    disconnect();
  }

  _transport->closeTCP();
  _state = ST_TCP_CLOSED;
}

/*!
 * \brief Connect to the MQTT server
 */
bool MQTT::connect()
{
  bool retval = false;
  if (_state != ST_TCP_OPEN) {
    if (!_transport->openTCP(_server, _port)) {
      goto ending;
    }
    _state = ST_TCP_OPEN;
  }

  // Assemble a CONNECT packet
  uint8_t pckt[MQTT_MAX_PACKET_LENGTH];
  size_t pckt_len;
  pckt_len = assembleConnectPacket(pckt, sizeof(pckt));
  if (!_transport->sendPacket(pckt, pckt_len)) {
    goto ending;
  }

  // Receive the CONNACK packet
  // Expecting CONNACK 20 02 00 00
  uint8_t mqtt_connack[4];
  if (!_transport->receivePacket(mqtt_connack, sizeof(mqtt_connack))) {
    goto ending;
  }
  if (mqtt_connack[0] != (CPT_CONNACK << 4) ||
      mqtt_connack[3] != 0) {
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
  bool retval = false;
  uint8_t pckt[2];
  pckt[0] = (CPT_DISCONNECT << 4);
  pckt[1] = 0;
  if (!_transport->sendPacket(pckt, 2)) {
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
size_t MQTT::assembleConnectPacket(uint8_t * pckt, size_t size)
{
  // Assume buf is not NULL
  // If name is not NULL then pw should be not NULL either.

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

  *pckt++ = (CPT_CONNECT << 4) | 0;
  // Assume length smaller than 128, or else we need multi byte length
  *pckt++ = pckt_len;

  // Variable header:
  //   Protocol Name,
  len = strlen(protocol_name);
  *pckt++ = highByte(len);
  *pckt++ = lowByte(len);
  memcpy(pckt, protocol_name, len);
  pckt += len;

  //   Protocol Level,
  *pckt++ = protocol_level;
  //   Connect Flags,
  uint8_t flags = 0;
  if (_name != 0 && _password != 0) {
    flags |= (1 << 7) | (1 << 6);
  }
  flags |= (1 << 1);            // clean session
  *pckt++ = flags;
  //   and Keep Alive (60 seconds)
  *pckt++ = 0x00;
  *pckt++ = 0x3C;

  len = strlen(_clientId);
  *pckt++ = highByte(len);
  *pckt++ = lowByte(len);
  memcpy(pckt, _clientId, len);
  pckt += len;

  len = strlen(_name);
  *pckt++ = highByte(len);
  *pckt++ = lowByte(len);
  memcpy(pckt, _name, len);
  pckt += len;

  len = strlen(_password);
  *pckt++ = highByte(len);
  *pckt++ = lowByte(len);
  memcpy(pckt, _password, len);
  pckt += len;

  return pckt_len + 2;
}

/*!
 * \brief Assemble a PUBLISH packet
 *
 * \returns The size of the assembled packet.
 */
size_t MQTT::assemblePublishPacket(uint8_t * pckt, size_t size,
    const char * topic, const uint8_t * msg, size_t msg_len)
{
  // Assume buf is not NULL

  int topic_length = strlen(topic);
  size_t pckt_len = 2 + topic_length + msg_len;
  if (size < (pckt_len + 2)) {
    // Oops. It does not fit. Truncate and hope for the best.
    pckt_len = size - 2;
  }

  *pckt++ = (CPT_PUBLISH << 4) | (1 << 0);         // DUP=0 | QoS=0 | RETAIN=1
  // Assume length smaller than 128, or else we need multi byte length
  *pckt++ = pckt_len;

  // 2 byte length of topic (MSB, LSB) followed by topic
  *pckt++ = highByte(topic_length);
  *pckt++ = lowByte(topic_length);
  memcpy(pckt, topic, topic_length);
  pckt += topic_length;
  memcpy(pckt, msg, msg_len);
  return pckt_len + 2;
}

MQTT mqtt;
