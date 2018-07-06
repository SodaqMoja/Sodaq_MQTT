# Sodaq_MQTT

This is an Arduino library to publish MQTT messages.  The communication
layer is abstracted so that different solutions can be added.

Currently it supports GPRSbee.

## Quick example

Here is a quick example (also present in the examples directory).  It is
not really exciting but it shows which minimal steps are required to
publish a message.  It uses a freely available MQTT server from the people
that wrote the mosquitto tools.  When you run this example you should set
up a client that listens (subscribes) to the messages.  Something like this:

    mosquitto_sub -h test.mosquitto.org -t "SODAQ/demo/#"

```c
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

```
