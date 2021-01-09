s#include <FS.h>
#include <SPIFFS.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ArduinoJson.h> 

WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient client(espClient);
char mqtt_server[50];
bool shouldSaveConfig = false;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

String conv(byte* s , int l)                                          //convert payload to string
{
  String ret;
  for (int i = 0 ; i < l ; i++)
    ret += char(s[i]);
  return ret;
}

void callback(char* topic, byte* payload, unsigned int length)        //pubusb callback
{
  String T = topic;
  if(T == "oximeter/rx")
    for(int i = 0; i < length ; i++)
      Serial.print((char)payload[i]);
}

void reconnect()
{
  Serial.print("Connecting to ");
  Serial.println(mqtt_server);
  if(client.connect(String(random(0xffff), HEX).c_str() , "oximeter/online" , 0 , true, "0"))
  {
    Serial.print("Connected");
    client.subscribe("oximeter/rx");
    client.publish("oximeter/online", "1" , true);
    client.publish("ack", "oximeter connected");
  }
}

void setup() {
  // initialize Serial:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(23, INPUT_PULLUP);
  pinMode(22, OUTPUT);
  pinMode(21, OUTPUT);
  digitalWrite(22, LOW);
  digitalWrite(21 ,LOW);
  
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  
  inputString.reserve(2000);

  Serial.println("mounting FS...");

  if (SPIFFS.begin(true)) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        StaticJsonDocument<200> doc;
        deserializeJson(doc , buf.get());
        serializeJson(doc, Serial);
        
        Serial.println("\nparsed json");
        strcpy(mqtt_server, doc["mqtt_server"]);
        
        configFile.close();
      }
    }
  } else Serial.println("failed to mount FS");
  
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setTimeout(180);
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.autoConnect("OLED Display");

  strcpy(mqtt_server, custom_mqtt_server.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    
    StaticJsonDocument<200> doc;
    doc["mqtt_server"] = mqtt_server;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    serializeJson(doc, Serial);
    serializeJson(doc, configFile);
    configFile.close();
    //end save
  }
  
  client.setCallback(callback);
  client.setServer(mqtt_server , 1883);
  client.setBufferSize(2015);
}

long long int mil;
int count = 0;

void loop() {
  digitalWrite(LED_BUILTIN , LOW);
  if(digitalRead(23))
    mil = millis();
  if(millis() - mil > 1500)
  {
    digitalWrite(LED_BUILTIN , HIGH);
    wifiManager.resetSettings();
    ESP.restart();
  }
  // print the string when a newline arrives:
  SerialEvent();
  if (stringComplete) {
    // clear the string:
    if(count >= 25)
    {
      count = 0;
      client.publish("oximeter/tx", inputString.c_str());
      inputString = "";
    }
    stringComplete = false;
  }
  if (!client.connected() && WiFi.status() == WL_CONNECTED)                     //reconnect/connect mqtt
    reconnect();
  client.loop();
}

/*
  SerialEvent occurs whenever a new data comes in the hardware Serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void SerialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') 
      count++;{
      stringComplete = true;
    }
  }
}
