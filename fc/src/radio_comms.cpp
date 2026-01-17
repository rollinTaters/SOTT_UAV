// First version of RC car code
// FOR THE CAR

#define SERIAL_DEBUG

#include <SPI.h>
#include "RF24.h"
#include "pin_config.hpp"
#include "comms_packets.hpp"


// instantiate an object for nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// adresses for the car/station pair
uint8_t uav_address[6] = { "av2st" };
uint8_t sta_address[6] = { "st2av" };


CommsPacket packet;
uint8_t payload[32] = {0};

uint32_t last_received_payload_time = 0;
uint8_t lost_signal_counter = 0;
uint16_t total_lost_signal_counter = 0;


// ==== SETUP ====
void initRadio()
{
    // initialize the transceiver on the SPI bus
    if( !radio.begin() ){
    // we fucked up, enter infinite loop (much good error handling)

        while(true)
        {      
            // TODO implement an actual error handling
            delay(100);
        };
    }

    // DEV OPTION -- set the PA level to low, close to each other modules cause power supply problems
    radio.setPALevel( RF24_PA_MIN ); // RF24_PA_MAX is default
    radio.setDataRate( RF24_250KBPS );
    radio.setChannel(0x25);

    radio.setPayloadSize(32);

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe( uav_address );     // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe( 1, sta_address );  // using pipe 1

    radio.startListening(); // put radio into RX mode  

}
// ==== End of Setup ====


// ==== Loop ====
void doRadioStuff()
{
    // we are a RX node
    uint8_t pipe;
    if( radio.available( &pipe ) )  // if there is a payload, get the pipe number that received it
    {
        uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
        radio.read( &payload, bytes );    // fetch payload from FIFO

        // TODO decode payload
        // TODO take action

        last_received_payload_time = millis();
        lost_signal_counter = 0;
    }

    // loss of signal detection
    if( (millis() - last_received_payload_time) > 1000 )
    {
        // TODO we lost contact
    }
}




/*
///// arduino code snippet for converting binary to hex ascii

uint8_t myData[32]; // Your raw data

void loop() {
  // Fill myData with some values...
  
  for (int i = 0; i < 32; i++) {
    if (myData[i] < 16) Serial.print('0'); // Leading zero
    Serial.print(myData[i], HEX);
  }
  Serial.println(); // The crucial "end of frame" marker
  delay(100);
}
*/
