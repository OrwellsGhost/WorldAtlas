// Authors: Richard B. Johnson
// Course:  CMSC838F Spring 2014 @ UMD
// Project: MPA04

#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

namespace Comm {
  static WiFiUDP Udp;
  static int status = WL_IDLE_STATUS;
  static const unsigned int localPort = 2390;
  static unsigned int remotePort = NULL;
  static IPAddress remoteIP;
  static IPAddress broadcastIP;
  static const byte BUFFER_SIZE = 32;
  static char buffer[BUFFER_SIZE];
  
  boolean connect(char* ssid, char* pass);
  void broadcast(char* s);
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
  
  IPAddress local = WiFi.localIP();
  IPAddress subnet = WiFi.subnetMask();
  
  broadcastIP[0] = local[0] | (~subnet[0]);
  broadcastIP[1] = local[1] | (~subnet[1]);
  broadcastIP[2] = local[2] | (~subnet[2]);
  broadcastIP[3] = local[3] | (~subnet[3]);
  
  Serial.print("Connected as ");
  Serial.println(local);
  
  Udp.begin(localPort);
  
  return true;
}

void Comm::broadcast(char* s) {
  Udp.beginPacket(broadcastIP, 2391);
  Udp.write(s);
  Udp.endPacket();
}

void Comm::send(char* s) {
  if (status != WL_CONNECTED)
    return;
    
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(s);
  Udp.endPacket();
  
  Serial.print(">> [Sent: ");
  Serial.print(remoteIP);
  Serial.print(":");
  Serial.print(remotePort);
  Serial.print("] ");
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
  
  Serial.print("<< [Rcvd: ");
  Serial.print(remoteIP);
  Serial.print(":");
  Serial.print(localPort);
  Serial.print("] ");
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
  const unsigned long baseFrameTimePeriod = 50;
  unsigned long lastFrameTimeStamp = millis();
  
  // Row to led output pin mapping
  int ledPins[globeHeight];
  
  int loc = 0; // the current column index
  int adj = 0; // spin right (+) or spin left (-)

  boolean hasPeer = false;
  
  int reedIndex = 0;
  int reedState = 1;
  unsigned long reeds[] = {0,0,0,0,0,0,0,0};
  float rpm = 0.0;
  
  void mapPinsToMega2560();
  void initReedSwitch();
  
  boolean establishPeer();
  boolean updateReeds();
  
  void lightColumn();
  void lightColumnAndPrint();
  void setColumn(boolean b);
  
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

void Atlas::initReedSwitch() {
  pinMode(17,INPUT);
  digitalWrite(17,1);
  
}

boolean Atlas::establishPeer() {
  char* s = Comm::receive();

  if (String(s) != "Echo") {
    //Comm::broadcast("Gamma");
    return false;
  }
  
  Comm::send("Tango");
  return true;
}

boolean Atlas::updateReeds() {
  int i, a, b, r = digitalRead(17);
  unsigned long s;
  
  if (r ^ reedState == 0)
    return false;
    
  reedState = r;
  
  if (r == 1)
    return false;
    
  reeds[reedIndex] = millis();
  reedIndex = (reedIndex + 1) & 7;
  
  for (i = 0, s = 0, a = reedIndex; i < 6; i++) {
    b = (a + 1) & 7;
    s += reeds[b] - reeds[a];
    a = b;
  }
 
  rpm = (s > 0) ? (180000.0 / float(s)) : 0.0;
  
  return true;
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

void Atlas::setColumn(boolean b) {
  int i;
  
  for (i = 0; i < globeHeight; i++) {
    digitalWrite(ledPins[i], b ? HIGH : LOW);
  }
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
  boolean updated = Atlas::updateReeds();
  
  if (hasPeer == false)
    hasPeer = Atlas::establishPeer();
    
  if (hasPeer && updated) {
    char c[32];
    dtostrf(rpm, 3, 4, c);
    Comm::send(c);
  }
  
  //Atlas::setColumn(reedState == 0);
  
  lightColumn();
  advance();
  sleep();
}

void setup() {
  Serial.begin(57600);
  Atlas::mapPinsToMega2560();
  Atlas::initReedSwitch();
  
  Comm::connect("SSID", "PSK");
}

void loop() {
  Atlas::operate();
}

