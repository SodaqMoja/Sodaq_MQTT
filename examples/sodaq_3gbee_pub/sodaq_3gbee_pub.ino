/*!
 * \file sodaq_3gbee_pub.ino
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

#include <Sodaq_3Gbee.h>
#include <Sodaq_MQTT.h>

#define APN "public4.m2minternet.com"
//#define APN "aerea.m2m.com"

static int counter;

#if defined(ARDUINO_SODAQ_AUTONOMO)
#define modemSerial Serial1
#define MySerial Serial2
#elif defined(ARDUINO_SODAQ_MBILI)
#define modemSerial Serial1
#define MySerial Serial
#error "Please select Autonomo or Mbili"
#endif

void setup()
{
    delay(2000);

    // We'll use this to print some messages
    MySerial.begin(57600);
    MySerial.println("test_mqtt");

    // Set the MQTT server hostname, and the port number
    mqtt.setServer("test.mosquitto.org", 1883);

    // OPTIONAL. Set the user name and password
    //mqtt.setAuth("Hugh", "myPass");

    // Set the MQTT client ID
    mqtt.setClientId("gprsbee_pub_12345");

    /*
     * The transport layer is a 3Gbee
     */
    // Beeslot
    modemSerial.begin(9600);

    // This is the code to initialize the 3Gbee
    sodaq_3gbee.init(modemSerial, BEE_VCC, BEEDTR, BEECTS);
    // Optionally enable diagnostic messages from GPRSbee
    sodaq_3gbee.setDiag(MySerial);

    // Set the APN. You can set the APN user and password too
    sodaq_3gbee.setApn(APN);
    // Inform our mqtt instance that we use gprsbee as the transport
    mqtt.setTransport(&sodaq_3gbee);
}

void loop()
{
    const char * topic = "SODAQ/demo/text";
    String msg = "Our message, number " + String(counter);
    // PUBLISH something
    if (!mqtt.publish(topic, msg.c_str())) {
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
