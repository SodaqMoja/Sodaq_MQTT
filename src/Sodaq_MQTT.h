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

#include "Sodaq_MQTT_Transport.h"

/*!
 * \brief The maximum length of a packet
 */
#define MQTT_MAX_PACKET_LENGTH  100

class MQTT
{
public:
  MQTT();
  void setServer(const char * server, uint16_t port = 1883);
  void setAuth(const char * name, const char * pw);
  void setClientId(const char * id);
  void setTransport(MQTT_Transport * transport) { _transport = transport; }
  bool publish(const char * topic, const uint8_t * msg, size_t msg_len);
  bool publish(const char * topic, const char * msg);
  void close();
private:
  bool connect();
  bool disconnect();
  size_t assemblePublishPacket(uint8_t * pckt, size_t size,
      const char * topic, const uint8_t * msg, size_t msg_len);
  size_t assembleConnectPacket(uint8_t * pckt, size_t size);
  //size_t assembleDisconnectPacket(uint8_t * pckt, size_t size);

  enum ControlPacketType_e {
    CPT_CONNECT = 1,
    CPT_CONNACK = 2,
    CPT_PUBLISH = 3,
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
  MQTT_Transport * _transport;
  const char * _server;
  uint16_t _port;
  const char * _name;
  const char * _password;
  const char * _clientId;
};

/*!
 * \brief This is the default (the only?) instance of the MQTT class
 */
extern MQTT mqtt;

#endif /* SODAQ_MQTT_H_ */
