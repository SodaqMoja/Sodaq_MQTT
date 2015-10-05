/*!
 * \file gprsbee_pub.ino
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

/*
 * This example shows how to do a MQTT PUBLISH via a GPRSbee connection.
 * It does the PUBLISH ten times and then it quits. The PUBLISH goes to
 * the test MQTT server at test.moquitto.org.
 *
 * To see the message you can subscribe, for example from the command
 * line with:
 *    mosquitto_sub -h test.mosquitto.org -t "SODAQ/demo/#"
 *
 * To build this example you need two Arduino libraries: Sodaq_MQTT and GPRSbee.
 */

#include <GPRSbee.h>
#include <Sodaq_MQTT.h>
#include <Sodaq_MQTT_GPRSbee.h>

static int counter;

void setup()
{
  // We'll use this to print some messages
  Serial.begin(9600);

  // Set the MQTT server hostname, and the port number
  mqtt.setServer("test.mosquitto.org", 1883);

  // OPTIONAL. Set the user name and password
  //mqtt.setAuth("Hugh", "myPass");

  // Set the MQTT client ID
  mqtt.setClientId("gprsbee_pub_12345");

  /*
   * The transport layer is a GPRSbee
   */
  // GPRSbee is connected to Serial1 on SODAQ Mbili
  Serial1.begin(9600);
  // This is the code to initialize SODAQ Mbili
  gprsbee.init(Serial1, BEECTS, BEEDTR, 200);
  // The power jumper of GPRSbee is connected to JP2
  gprsbee.setPowerSwitchedOnOff(true);
  // Optionally enable diagnostic messages from GPRSbee
  // gprsbee.setDiag(Serial);

  // Set the APN. You can set the APN user and password too
  mqtt_gprsbee.setApn("public4.m2minternet.com");
  // Inform our mqtt instance that we use mqtt_gprsbee as the transport
  mqtt.setTransport(&mqtt_gprsbee);
}

void loop()
{
  const char * topic = "SODAQ/demo/text";
  const char * msg = "Our message";
  // PUBLISH something
  if (!mqtt.publish(topic, msg)) {
    Serial.println("publish failed");
    while (true) {}
  }

  ++counter;
  if (counter >= 10) {
    mqtt.close();
    // End of the demo. Wait here for ever
    while (true) {}
  }

  // Wait a little.
  delay(3000);
}
