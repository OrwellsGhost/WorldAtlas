// Authors: Richard B. Johnson
// Course:  CMSC838F Spring 2014 @ UMD
// Project: MPA04

// THIS PROJECT IS STILL IN WORKING PROGRESS

#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

namespace Comm {
  static WiFiUDP Udp;
  static int status = WL_IDLE_STATUS;
  static const unsigned int localPort = 2390;
  static unsigned int remotePort = NULL;
  static IPAddress remoteIP;
  static const byte BUFFER_SIZE = 32;
  static char buffer[BUFFER_SIZE];
  
  boolean connect(char* ssid, char* pass);
  void send(char* s);
  char* receive();
}

boolean Comm::connect(char* ssid, char* pass) {
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present.");
    return false;
  } 
  
  Serial.print("Attempting to connect to: ");
  Serial.println(ssid);
  
  status = WiFi.begin(ssid, pass);
  
  if (status != WL_CONNECTED) {
    Serial.print("Connection failed.");
    return false;
  }
  
  delay(1000);
  
  Serial.print("Connected as ");
  Serial.println( WiFi.localIP() );
  
  Udp.begin(localPort);
  
  return true;
}

void Comm::send(char* s) {
  if (status != WL_CONNECTED)
    return;
    
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(s);
  Udp.endPacket();
  
  Serial.print("Sent: ");
  Serial.print(remoteIP);
  Serial.print(":");
  Serial.print(remotePort);
  Serial.print(" ");
  Serial.println(s);
}

char* Comm::receive() {
  int len;
  
  if (status != WL_CONNECTED)
    return NULL;
  
  len = Udp.parsePacket();
  
  if (len == 0)
    return NULL;
  
  len = Udp.read(buffer, BUFFER_SIZE);
  
  if (len < 1 || len > BUFFER_SIZE)
    return NULL;
  
  remoteIP = Udp.remoteIP();
  remotePort = Udp.remotePort();
  
  buffer[len] = 0;
  
  Serial.print("Rcvd: ");
  Serial.print(remoteIP);
  Serial.print(":");
  Serial.print(localPort);
  Serial.print(" ");
  Serial.println(buffer);
  
  return buffer;
}


namespace Atlas {
  // Index: column from left to right with increase
  // Value: from top (low bit) to bottom (high bit)
  static const unsigned long globe[] = {
    0x00000000,0x00000000,0x00000018,0x00000008,
    0x0000000c,0x0000070c,0x00000ffc,0x00003ffc,
    0x00007ffc,0x00004ffc,0x0000c7fc,0x000087fc,
    0x00180ffe,0x003f277e,0x007f03ec,0x1fff01c4,
    0x7fff00e4,0x9ffe00f2,0x0ffc003a,0x03f80065,
    0x01f80000,0x00f00001,0x0010000d,0x0000000f,
    0x00000007,0x00000007,0x00000003,0x0001f009,
    0x0003f800,0x0003ff20,0x0001fde0,0x0001fcc0,
    0x0003fcf8,0x00fff9d8,0x07fff9e4,0x07fff9fc,
    0x07fffafc,0x01fffaf8,0x007fdef8,0x0007bffc,
    0x0183fe7c,0x00416dfc,0x00006ffc,0x00001ffc,
    0x00001ffe,0x00001ffe,0x00003ffe,0x0001fffe,
    0x00007ffe,0x00003ffc,0x00001ffe,0x00007ffc,
    0x0004fffc,0x0008fffc,0x0000bffc,0x070c1dfc,
    0x070619dc,0x078800cc,0x07c0040c,0x07c0030c,
    0x0fc8002c,0x0f980008,0x07d00000,0x02000000,
    0x00000000,0x00000000,0x00000000,0x00000000,
    0x00000000,0x00000000,0x00000000,0x00000000
  };
  
  // 72 columns -> 5 degrees latitude gradient
  const int globeWidth = sizeof(globe) >> 2;
  const int globeHeight = 32;
  
  // Base frame time: (milliseconds)
  // 120ms ->  500 rpm,  8.3 fps
  // 100ms ->  600 rpm, 10.0 fps
  //  75ms ->  800 rpm, 13.3 fps
  //  60ms -> 1000 rpm, 16.7 fps
  //  50ms -> 1200 rpm, 20.0 fps
  //  40ms -> 2000 rpm, 25.0 fps
  //  20ms -> 3000 rpm, 50.0 fps
  const unsigned long baseFrameTimePeriod = 20;
  unsigned long lastFrameTimeStamp = millis();
  
  // Row to led output pin mapping
  int ledPins[globeHeight];
  
  int loc = 0; // the current column index
  int adj = 0; // spin right (+) or spin left (-)
  
  void mapPinsToMega2560();
  void lightColumn();
  void lightColumnAndPrint();
  int  sign(int x);
  void advance();
  void sleep();
  void operate();
}

void Atlas::mapPinsToMega2560() {
  for (int i = 0; i < globeHeight; i++) {
    ledPins[i] = 18 + i;
    pinMode(ledPins[i], OUTPUT);     
  }
}

void Atlas::lightColumn() {
  unsigned long mask;
  unsigned long col = globe[loc];
  int i;
  
  for (i = 0, mask = 1; i < globeHeight; i++, mask <<= 1) {
    digitalWrite(ledPins[i], (col & mask) ? HIGH : LOW);
  }
}

void Atlas::lightColumnAndPrint() {
  unsigned long mask;
  unsigned long col = globe[loc];
  String s = "";
  int i;
  
  for (i = 0, mask = 1; i < globeHeight; i++, mask <<= 1) {
    digitalWrite(ledPins[i], (col & mask) ? HIGH : LOW);
    s += (col & mask) ? "1" : "0";
  }
  
  Serial.println(s);
}

int Atlas::sign(int x) {
  return ((x & 0x8000) ? -1 : 1);
}

void Atlas::advance() {
  loc = (loc + sign(adj)) % globeWidth;
}

void Atlas::sleep() {
  unsigned long elapsedTimePeriod = millis() - lastFrameTimeStamp;
  unsigned long remainingTimePeriod = baseFrameTimePeriod - elapsedTimePeriod;
  //unsigned long timePeriodAdjustment = min(abs(adj), remainingTimePeriod - 1);
  
  if (baseFrameTimePeriod > elapsedTimePeriod) {
    delay(remainingTimePeriod);
  }
  
  lastFrameTimeStamp = millis();
}

void Atlas::operate() {
  char* s = Comm::receive();
  
  if (String(s) == "Echo")
    Comm::send("Tango");
    
  lightColumn();
  advance();
  sleep();
}

void setup() {
  Serial.begin(57600);
  Atlas::mapPinsToMega2560();
  //Comm::connect("ENTER HERE", "ENTER HERE");
  
  //pinMode(16, INPUT);
  //digitalWrite(16, HIGH);
}

void loop() {
  Atlas::operate();
  //Serial.println(digitalRead(16));
}

