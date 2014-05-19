// Authors: Richard B. Johnson
// Course:  CMSC838F Spring 2014 @ UMD
// Project: MPA04

// THIS PROJECT IS STILL IN WORKING PROGRESS

import hypermedia.net.*;

UDP udp;
String remoteIP = null;
int remotePort = -1;

void find() {
  udp.send("Echo", "192.168.0.255", 2390);
}

void receive(byte[] message, String ip, int port) {
  if (ip.equals("127.0.0.1")) return;
  
  print("Rcvd: " + ip + ":" + port + " ");
  println(new String(message));
  
  remoteIP = ip;
  remotePort = port;
  
  udp.send("Bravo", ip, port);
  udp.broadcast(false);
}

public void setup() {
  size(400, 300);
  
  udp = new UDP(this, 2391);
  udp.broadcast(true);
  udp.listen(true);
}

public void draw() {
  find();
}

