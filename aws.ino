#include <FS.h> //this needs to be first, or it all crashes and burns..
#include <ESP8266WiFi.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoJson.h>
#include <Arduino.h>
#include <Stream.h>
#include <ESP8266WiFiMulti.h>#include <Wire.h>
#include <energyic_UART.h>
#include <Wire.h>
  
#if !defined(ARDUINO_ARCH_SAMD)
#include <SoftwareSerial.h>8
#else
#endif

//AWS
#include <sha256.h>
#include "Utils.h"

//WEBSockets
#include <Hash.h>
#include <WebSocketsClient.h>

//MQTT PAHO
#include <SPI.h>
#include <IPStack.h>
#include <Countdown.h>
#include <MQTTClient.h>



//AWS MQTT Websocket
#include "Client.h"
#include "AWSWebSocketClient.h"
#include "CircularByteBuffer.h"

//flag for saving data
bool shouldSaveConfig = false;



#if defined(ESP8266)
//NOTE: Version 1.0 and 1.1 of featherwing use pins 14,12
//version 1.2 and above using pins 13,14
//SoftwareSerial ATMSerial(14, 12, false, 256); //RX, TX v1.0-1.1
SoftwareSerial ATMSerial(13, 14, false, 256); //RX, TX v1.2+
//SoftwareSerial ATMSerial(D4, D3, false, 256); //NodeMCU v1.0
#endif

#ifdef AVR_ATmega32U4 //32u4 board
SoftwareSerial ATMSerial(11, 13); //RX, TX
#endif

#if defined(ARDUINO_ARCH_SAMD)
#include "wiring_private.h" // pinPeripheral() function
//Feather M0
#define PIN_SerialATM_RX       12ul
#define PIN_SerialATM_TX       11ul
#define PAD_SerialATM_RX       (SERCOM_RX_PAD_3)
#define PAD_SerialATM_TX       (UART_TX_PAD_0)

// Using SERCOM1 on M0 to communicate with ATM90E26
Uart ATMSerial(&sercom1, PIN_SerialATM_RX, PIN_SerialATM_TX, PAD_SerialATM_RX, PAD_SerialATM_TX);
#endif

ATM90E26_UART eic(&ATMSerial);
WiFiClient client;
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

struct clientData {
  int current[8];
  char  voltage[8];
  char realpower[8];
  char powerfactor[8];
  char frequency[8];
};

int conv;
void MQTT_connect();


void setup(){
 char auth[36] = "AKIA45IPCKAKD3J7OKID";
 char server[50] = "a2geapajmwonm2-ats.iot.us-east-2.amazonaws.com";
 //int x = 0;
  Serial.begin(115200);
#if defined(ARDUINO_ARCH_SAMD)
  pinPeripheral(PIN_SerialATM_RX, PIO_SERCOM);
  pinPeripheral(PIN_SerialATM_TX, PIO_SERCOM);
#endif
  //Must begin ATMSerial before IC init
  ATMSerial.begin(9600);
  eic.InitEnergyIC();

Wire.begin();

  //Read previous config

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_ts_token("ts", "AKIA45IPCKAKD3J7OKID", auth, 33);
  WiFiManagerParameter custom_server("serv", "a2geapajmwonm2-ats.iot.us-east-2.amazonaws.com", server, 50);

  //Use wifi manager to get config
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_ts_token);
  wifiManager.addParameter(&custom_server);

  //first parameter is name of access point, second is the password
 wifiManager.autoConnect("Minion", "password");

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  Serial.print("Key:");
  Serial.println(auth);
  Serial.print("Server:");
  Serial.println(server);

  //read updated parameters
  strcpy(auth, custom_ts_token.getValue());
  strcpy(server, custom_server.getValue());

  delay(1000);

}

char aws_endpoint[]    = "a2geapajmwonm2-ats.iot.us-east-2.amazonaws.com";
char aws_key[]         = "AKIA45IPCKAKD3J7OKID";
char aws_secret[]      = "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";
char aws_region[]      = "us-east-2";
const char* aws_topic  = "$aws/things/minion_new/shadow/update";
int port = 443;


//MQTT config
const int maxMQTTpackageSize = 512;
const int maxMQTTMessageHandlers = 1;

ESP8266WiFiMulti WiFiMulti;

AWSWebSocketClient awsWSclient(1000);

IPStack ipstack(awsWSclient);
MQTT::Client<IPStack, Countdown, maxMQTTpackageSize, maxMQTTMessageHandlers> *client = (NULL);

//# of connections
long connection = 0;

//generate random mqtt clientID
char* generateClientID () {
  char* cID = new char[23]();
  for (int i=0; i<22; i+=1)
    cID[i]=(char)random(1, 256);
  return cID;
}

//count messages arrived
  float id = ESP.getChipId();
  float Vrms1 = eic.GetLineVoltage();
  float Crms1 = eic.GetLineCurrent();
  float realPower1 = eic.GetActivePower();
  float powerFactor1 = eic.GetPowerFactor();
  float freq1 = eic.GetFrequency();

//callback to handle mqtt messages
void messageArrived(MQTT::MessageData& md)
{
  MQTT::Message &message = md.message;

  Serial.print(id);
  Serial.print(",");
  Serial.print(Vrms1);
  Serial.print(",");
  Serial.print(Crms1);
  Serial.print(",");
  Serial.print(realPower1);
  Serial.print(",");
  Serial.print(powerFactor1);
  Serial.print(",");
  Serial.print(freq1);
 // char* msg = new char[message.payloadlen+1]();
  //memcpy (msg,message.payload,message.payloadlen);
  Serial.println(msg);


//--------------------------------------------------------------------------

 // char JSONMessage[] = "{\"state\":{\"reported\":{\"test_value1\":297, \"test_value2\":123}}}";
  Serial.print("Initial string value: ");
  Serial.println(msg);
 
 StaticJsonBuffer<300> JSONBuffer;   //Memory pool
  JsonObject& parsed = JSONBuffer.parseObject(msg); //Parse message
 
  if (!parsed.success()) {   //Check for errors in parsing
 
    Serial.println("Parsing failed");
    delay(5000);
    return;
 
  }
 
//  const char * sensorType = parsed["SensorType"]; //Get sensor type value
  ///int value = parsed["Value"];                                         //Get value of sensor measurement
 
//  Serial.println(sensorType);
// Serial.println(value);

 // const char* id = parsed["state"]["reported"]["test_value1"];
  const char* current = parsed["ampere"]["reported"]["test_value2"];  
  //const char* voltage = parsed["state"]["reported"]["test_value3"];  
  //const char* realpower = parsed["state"]["reported"]["test_value4"]; 
  //const char* powerfactor = parsed["state"]["reported"]["test_value5"]; 
      
  int conv=atoi(current);
  Serial.print("The concertvalue is ");
Serial.println(conv);
/*
int conv=atoi(id);
  Serial.print("The concertvalue is ");
Serial.println(conv);

int conv=atoi(voltage);
  Serial.print("The concertvalue is ");
Serial.println(conv);

int conv=atoi(realpower);
  Serial.print("The concertvalue is ");
Serial.println(conv);
*/
   Serial.println(conv);


  //-------------------------------------------------------------------------
 /*
  if(conv==290)
  {
    digitalWrite(16,HIGH);
    
    }
    else if(conv==200)
    {
      digitalWrite(16,LOW);
      
      }
  delete msg;
  */
}

//connects to websocket layer and mqtt layer
bool connect () {

    if (client == NULL) {
      client = new MQTT::Client<IPStack, Countdown, maxMQTTpackageSize, maxMQTTMessageHandlers>(ipstack);
    } else {

      if (client->isConnected ()) {    
        client->disconnect ();
      }  
      delete client;
      client = new MQTT::Client<IPStack, Countdown, maxMQTTpackageSize, maxMQTTMessageHandlers>(ipstack);
    }


    //delay is not necessary... it just help us to get a "trustful" heap space value
    delay (1000);
    Serial.print (millis ());
    Serial.print (" - conn: ");
    Serial.print (++connection);
    Serial.print (" - (");
    Serial.print (ESP.getFreeHeap ());
    Serial.println (")");




   int rc = ipstack.connect(aws_endpoint, port);
    if (rc != 1)
    {
      Serial.println("error connection to the websocket server");
      return false;
    } else {
      Serial.println("websocket layer connected");
    }


    Serial.println("MQTT connecting");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    char* clientID =generateClientID ();
    data.clientID.cstring = clientID;
    rc = client->connect(data);
    //delete[] clientID;
    if (rc != 0)
    {
      Serial.print("error connection to MQTT server");
      Serial.println(rc);
      return false;
    }
    Serial.println("MQTT connected");
    return true;
}

//subscribe to a mqtt topic
void subscribe () {
   //subscript to a topic
    int rc = client->subscribe(aws_topic, MQTT::QOS0, messageArrived);
    if (rc != 0) {
      Serial.print("rc from MQTT subscribe is ");
      Serial.println(rc);
      return;
    }
    Serial.println("MQTT subscribed");
    
    
}

//send a message to a mqtt topic
void sendmessage () {
    //send a message
    MQTT::Message message;
    char buf[100];
    strcpy(buf, "{\"state\":{\"reported\":{\"test_value1\":299,\"test_value2\":292}}}");
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*)buf;
    message.payloadlen = strlen(buf)+1;
    int rc = client->publish(aws_topic, message); 
}


void setup() {
    Serial.begin (115200);
    delay (2000);
    Serial.setDebugOutput(1);
    pinMode(16,OUTPUT);
    //fill with ssid and wifi password
    WiFiMulti.addAP(wifi_ssid, wifi_password);
    Serial.println ("connecting to wifi");
    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
        Serial.print (".");
    }
    Serial.println ("\nconnected");

    //fill AWS parameters    
    awsWSclient.setAWSRegion(aws_region);
    awsWSclient.setAWSDomain(aws_endpoint);
    awsWSclient.setAWSKeyID(aws_key);
    awsWSclient.setAWSSecretKey(aws_secret);
    awsWSclient.setUseSSL(true);
    configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

    if (connect ()){
      subscribe ();
      sendmessage ();
    }

}

void loop() {

messageArrived;
  
  //keep the mqtt up and running
  if (awsWSclient.connected ()) {    
      client->yield();
  } else {
    //handle reconnection
    if (connect ()){
      subscribe ();      
    }
  }

}
