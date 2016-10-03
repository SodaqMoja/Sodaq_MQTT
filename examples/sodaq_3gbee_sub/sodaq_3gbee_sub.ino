/*!
 * \file sodaq_3gbee_sub.ino
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
 * This example shows how to do a MQTT SUBSCRIBE via a 3Gbee connection.
 * It subscribes to SODAQ/demo/# and prints incoming messages.
 *
 * To see a message you can publish, for example from the command
 * line with:
 *    mosquitto_pub -h test.mosquitto.org -t 'SODAQ/demo/test' -m 'hello world'
 *
 * To build this example you need the following Arduino libraries:
 *  - Sodaq_MQTT,
 *  - Sodaq_3Gbee and
 *  - Sodaq_wdt.
 */


#include <Arduino.h>
#include <Sodaq_3Gbee.h>
#include <Sodaq_MQTT.h>
#include <Sodaq_wdt.h>

//#define APN "public4.m2minternet.com"
#define APN "aerea.m2m.com"

static bool is_subscribed;

#if defined(ARDUINO_SODAQ_AUTONOMO)
#define modemSerial Serial1
#define MySerial Serial
#elif defined(ARDUINO_SODAQ_MBILI)
#define modemSerial Serial1
#define MySerial Serial
#else
#error "Please select Autonomo or Mbili"
#endif

static void handlePublish(const char *topic, const uint8_t *msg, size_t msg_length);
static void handlePacket(uint8_t *pckt, size_t len);
static void fatal();

void setup()
{
    // In case of reset (this is probably unnecessary)
    sodaq_wdt_disable();

    delay(3000);

    // We'll use this to print some messages
    MySerial.begin(57600);
    MySerial.println("test_mqtt");

    // Set the MQTT server hostname, and the port number
    mqtt.setServer("test.mosquitto.org", 1883);

    // OPTIONAL. Set the user name and password
    //mqtt.setAuth("Hugh", "myPass");

    // Set the MQTT client ID
    mqtt.setClientId("sodaq_12345");

    // Beeslot
    Serial1.begin(9600);

#if defined(ARDUINO_SODAQ_AUTONOMO)
    sodaq_3gbee.init(modemSerial, BEE_VCC, BEEDTR, BEECTS);
#endif
    sodaq_3gbee.setDiag(MySerial);
    sodaq_3gbee.setFlushEverySend();

    sodaq_3gbee.setApn(APN);
    sodaq_3gbee.setApnUser("user");
    sodaq_3gbee.setApnPass("pass");
    mqtt.setTransport(&sodaq_3gbee);

    mqtt.setDiag(MySerial);
    mqtt.setPublishHandler(handlePublish);
    mqtt.setPacketHandler(handlePacket);

    mqtt.setKeepAlive(5 * 60);

    // Enable Watch Dog Timer ??
    //sodaq_wdt_enable();
}

static uint32_t prevPing = millis();
void loop()
{

    // Handle incoming SUBSCRIBE?

    if (mqtt.loop()) {
        // An incoming packet was handled
        // Don't do anything else in this anymore.
        return;
    }

    if (!mqtt.isConnected()) {
        is_subscribed = false;
    }

    if (!is_subscribed) {
        if (!mqtt.subscribe("SODAQ/demo/test")) {
            MySerial.println("subscribe failed");
        } else {
            is_subscribed = true;
        }
    }

    uint32_t now = millis();
    int32_t since = now - prevPing;
    if (since > (1000L * 4 * 60)) {
        // Send PINGREQ
        if (!mqtt.ping()) {
            MySerial.println("ping failed");
        } else {
        }
        prevPing = now;
    }
}

/*
 * Handler for incoming PUBLISH packets
 */
static void handlePublish(const char *topic, const uint8_t *msg, size_t msg_length)
{
    MySerial.println(String("Incoming PUBLISH, topic=") + topic);
    // Hopefully the string is terminated with a NUL byte
    MySerial.println(String("   msg=") + (const char *)msg);
}

/*
 * Generic MQTT packet handler
 *
 * This handler is called when mqtt.loop sees a packet
 * and it is not handled otherwise.
 */
static void handlePacket(uint8_t *pckt, size_t len)
{
}

static void fatal()
{
    sodaq_3gbee.off();
    while (true) {
    }
}
