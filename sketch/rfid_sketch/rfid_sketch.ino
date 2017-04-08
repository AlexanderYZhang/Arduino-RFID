/*
RFID Reader that checks RFID tags against a database on a computer and
turns a servo if it is a valid tag in the database. See 
http://appdelegateinc.com/blog/2010/10/06/rfid-auth-using-arduino-and-python
for more details.

MIT License - Share/modify/etc, but please keep this notice.

Copyright (c) 2010 Matt Williamson, App Delegate Inc

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// RFID Settings
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "SparkFun_UHF_RFID_Reader.h"
#define BUZZER1 9
#define BUZZER2 10
SoftwareSerial softSerial(2, 3);
RFID nano;

// These are used for serial communication
int val = 0; 
char rfid[10];
int bytesRead = 0;
boolean start = true;
int pinLed = 13;

void emptySerialBuffers() {
   // This empties the serial buffers to prevent repeat reads
   while(softSerial.available() > 0) {
        softSerial.read();
    }
    while(Serial.available() > 0) {
        Serial.read();
    }
}


//From example code
//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.begin(softSerial); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while(!softSerial); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while(softSerial.available()) softSerial.read();
  
  nano.getVersion();

  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano.stopReading();

    Serial.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    softSerial.begin(115200); //Start software serial at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
  }

  //Test the connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2

  nano.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}

void setup() {
    // Serial to computer
    Serial.begin(115200);

    pinMode(BUZZER1, OUTPUT);
    pinMode(BUZZER2, OUTPUT);
    
    // Set up RFID Reader
    while (!Serial);

    
    if (setupNano(38400) == false) {
      Serial.println(F("Module failed to respon. Please check wiring."));
      while (1); //Stop
    }

    nano.setRegion(REGION_NORTHAMERICA);

    nano.setReadPower(500);

    nano.startReading();
}

void loop() {
    // Read serial from RFID reader
    if (nano.check() == true && start) {
      byte responseType = nano.parseResponse();
      if (responseType == RESPONSE_IS_KEEPALIVE) {
        Serial.println(F("Scanning"));
      } else if (responseType == RESPONSE_IS_TAGFOUND) {
        int rssi = nano.getTagRSSI();
        long freq = nano.getTagFreq();
        long timeStamp = nano.getTagTimestamp();
        byte tagEPCBytes = nano.getTagEPCBytes();
      
        Serial.print(F(" rssi["));
        Serial.print(rssi);
        Serial.print(F("]"));

        Serial.print(F(" freq["));
        Serial.print(freq);
        Serial.print(F("]"));

        Serial.print(F(" time["));
        Serial.print(timeStamp);
        Serial.print(F("]"));

        Serial.print(F(" epc["));
        for (byte x = 0; x < tagEPCBytes; x++) {
          if (nano.msg[31 + x], 0x10) Serial.print(F("0"));
          rfid[x] = nano.msg[31 + x];
          Serial.print(nano.msg[31 + x], HEX);
          Serial.print(F(" "));
        }
        emptySerialBuffers();
        Serial.print(F("]"));
        start = false;
        tone(BUZZER1, 2093, 150); //C
        delay(150);
        tone(BUZZER1, 2349, 150); //D
        delay(150);
        tone(BUZZER1, 2637, 150); //E
        delay(150);
        Serial.println(); 
        while (!Serial.available());
        Serial.read();
        start = true;
      } else if (responseType == ERROR_CORRUPT_RESPONSE) {
        Serial.println("Bad CRC");
      } else {
        Serial.print("Unknown error");
      }
    }
  }



