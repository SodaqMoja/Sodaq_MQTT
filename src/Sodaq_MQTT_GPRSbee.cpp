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

#include <GPRSbee.h>

#include "Sodaq_MQTT_GPRSbee.h"

void MQTT_GPRSbee::setApn(const char * apn, const char * apnUser, const char * apnPw)
{
  _apn = apn;
  _apnUser = apnUser;
  _apnPassword = apnPw;
}

bool MQTT_GPRSbee::openTCP(const char * server, uint16_t port)
{
  if (!gprsbee.on()) {
    return false;
  }
  if (!gprsbee.networkOn()) {
    return false;
  }
  return gprsbee.openTCP(_apn, _apnUser, _apnPassword, server, port);
}

bool MQTT_GPRSbee::closeTCP()
{
  gprsbee.closeTCP();
  return true;
}

bool MQTT_GPRSbee::sendPacket(uint8_t * pckt, size_t len)
{
  return gprsbee.sendDataTCP(pckt, len);
}

bool MQTT_GPRSbee::receivePacket(uint8_t * pckt, size_t expected_len)
{
  return gprsbee.receiveDataTCP(pckt, expected_len);
}
