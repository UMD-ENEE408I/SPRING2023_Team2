/*
 *  This sketch sends random data over UDP on a ESP32 device
 *
 */
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi network name and password:
const char * networkName = "408ITerps";
const char * networkPswd = "goterps2022";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress =  "192.168.2.143"; //my computer ipv4 address when im connect to 408iterps
const int udpPort = 3333; //port on laptop/jetson. needs to be the same in the python file

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;


//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}

//function to make sure you are connected.
void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}


void setup(){
  // Initilize hardware serial:
  Serial.begin(115200);
  
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
}

void loop(){ // what is being run/ equivalent ot main function
  //only send data when connected
  if(connected){
    //Send a packet
    udp.beginPacket(udpAddress,udpPort);
    udp.printf("Seconds since boot: %lu", millis()/1000);
    udp.endPacket();
  }

  //copied from team 2 from last year. it sends packets from laptop to mice
  int packetSize = udp.parsePacket();
  if(packetSize >= sizeof(float)){
    Serial.printf("packet size is %d\n", packetSize);
    float my_array[1]; 
    udp.read((char*)my_array, sizeof(my_array)); 
    udp.flush();
    Serial.printf("received value is %f\n", my_array[0]);
    float target = my_array[0];
    Serial.printf("target is %f\n", target);
  }


  //Wait for 1 second
  delay(100);
}



