# Sodaq_MQTT

This is an Arduino library to publish MQTT messages.  The communication
layer is abstracted so that different solutions can be added.

Currently it supports Sodaq_3Gbee.

## Quick example

Here is a quick example (also present in the examples directory).  It is
not really exciting but it shows which minimal steps are required to
publish a message.  It uses a freely available MQTT server from the people
that wrote the mosquitto tools.  When you run this example you should set
up a client that listens (subscribes) to the messages.  Something like this:

    mosquitto_sub -h test.mosquitto.org -t "SODAQ/demo/#"

```c
#include <Sodaq_3Gbee.h>
#include <Sodaq_MQTT.h>

#define APN "public4.m2minternet.com"
//#define APN "aerea.m2m.com"

static int counter;

#if defined(ARDUINO_SODAQ_AUTONOMO)
#define modemSerial Serial1
#define MySerial SerialUSB
#elif defined(ARDUINO_SODAQ_MBILI)
#define modemSerial Serial1
#define MySerial Serial
#else
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
        MySerial.println("publish failed");
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
