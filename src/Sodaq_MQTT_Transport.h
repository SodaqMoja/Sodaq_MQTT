/*!
 * \file Sodaq_MQTT_Transport.h
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

#ifndef SODAQ_MQTT_TRANSPORT_H_
#define SODAQ_MQTT_TRANSPORT_H_

#include <stddef.h>
#include <stdint.h>

/*!
 * \brief This class is used to define the interface functions to transport MQTT packets
 *
 * Use this class to send and receive packets with your own communication
 * software.  For example, create a derived class that uses GPRSbee for transport.
 */
class MQTT_Transport
{
public:
  virtual ~MQTT_Transport() {}

  virtual bool openTCP(const char * server, uint16_t port = 1883) = 0;
  virtual bool closeTCP() = 0;
  virtual bool sendPacket(uint8_t * pckt, size_t len) = 0;
  virtual bool receivePacket(uint8_t * pckt, size_t expected_len) = 0;
protected:
private:
};

#endif /* SODAQ_MQTT_TRANSPORT_H_ */
