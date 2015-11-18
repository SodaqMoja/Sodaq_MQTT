/*!
 * \file Sodaq_MQTT_GPRSbee.h
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

#ifndef SODAQ_MQTT_GPRSBEE_H_
#define SODAQ_MQTT_GPRSBEE_H_

#include <stddef.h>
#include <stdint.h>

#include "Sodaq_MQTT_Transport.h"

class MQTT_GPRSbee : public MQTT_Transport
{
public:
  ~MQTT_GPRSbee() {}

  bool openTCP(const char * server, uint16_t port = 1883);
  bool closeTCP(bool switchOff=true);
  bool sendPacket(uint8_t * pckt, size_t len);
  bool receivePacket(uint8_t * pckt, size_t expected_len);

  void setApn(const char * apn, const char * apnUser = 0, const char * apnPw = 0);
private:
  const char * _apn;
  const char * _apnUser;
  const char * _apnPassword;
};

/*!
 * \brief This is the default (the only?) instance of the MQTT_GPRSbee class
 */
extern MQTT_GPRSbee mqtt_gprsbee;

#endif /* SODAQ_MQTT_GPRSBEE_H_ */
