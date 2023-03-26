/*
  *     Internet Clock Firmware for ESP8266
  *     Author: Ashwin Vallaban     https://github.com/ashvnv
  *     Date: Feb 2023
*/

#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char *ssid     = "abcd";//Wifi SSID
const char *password = "Ashwin99";//Wifi password
const long utcOffsetInSeconds = 19800; // India +5:30 .. (5hr * 60min + 30min) * 60sec

WiFiUDP ntpUDP;

NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);


void setup(){
  Serial.begin(9600);//Set the Baud rate to 9600
  
  WiFi.begin(ssid, password);//Connect to the WiFi

  //Wait for the connection to complete
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
  }

  //Init the timeClient Object
  timeClient.begin();
}

//For converting a value to BCD format
byte int_to_bcd(int val){
    return (((val / 10) << 4) | (val % 10));
  }

void loop() {
  
  //Loop Makes sure that ESP-01 is connected to WiFi
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
  }

//Check if PIC sent any data
while (Serial.available()) {
    //data is sent by PIC. Receive it and ignore
    byte c = Serial.read();
    timeClient.update();//Update the time according to internet time
    Serial.write(int_to_bcd(timeClient.getHours())); //send hours in BCD
    Serial.write(int_to_bcd(timeClient.getMinutes())); //send mins in BCD
    Serial.write(int_to_bcd(timeClient.getSeconds())); //send seconds in BCD
}
}
