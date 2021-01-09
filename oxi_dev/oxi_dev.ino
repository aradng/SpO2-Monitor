/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include "brzo_i2c.h"
#include <Wire.h>
#include "MPU6050.h"
#include "SH1106.h"
#include "Adafruit_MCP9808.h"
#include "MAX30100.h"

SH1106Wire display(0x3c , 33 , 32);    //d1 d2

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

#define OUTPUT_READABLE_ACCELGYRO
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SAMPLING_RATE                       MAX30100_SAMPRATE_100HZ
#define IR_LED_CURRENT                      MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT                     MAX30100_LED_CURR_33_8MA
#define PULSE_WIDTH                         MAX30100_SPC_PW_1600US_16BITS
#define HIGHRES_MODE                        true
MAX30100 sensor;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

String var2str(int inp)
{
  String tmp , rtmp;
  inp = abs(inp);
  if(inp == 0)
    tmp += '0';
  while(inp != 0)
  {
    tmp += char('0'+ inp % 10);
    inp /= 10;
  }
  for(int i = tmp.length() ; i > 0 ; i--)
    rtmp += tmp[i - 1];
  return rtmp;
}

String var2str(double inp , int dig)
{
  inp = abs(inp);
  String tmp , rtmp;
  int inpi = int(inp);
  int inpr = (inp - inpi) * pow(10 , dig);
  for(int i = 0 ; i < dig ; i++)
  {
    tmp += char('0'+ inpr % 10);
    inpr /= 10;
  }
  if(dig > 0)
    tmp += '.';
  if(inpi == 0)
    tmp += '0';
  while(inpi != 0)
  {
    tmp += char('0'+ inpi % 10);
    inpi /= 10;
  }
  for(int i = tmp.length() ; i > 0 ; i--)
    rtmp += tmp[i - 1];
  return rtmp;
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        Serial.println("*********");
      }
    }
};


void setup() {
  
  pinMode(14 , INPUT_PULLUP);
  Serial.begin(500000);
  Wire1.begin(SDA, SCL);
  
  display.init();
  display.clear();
  display.setColor(WHITE);
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0 ,0 , "Starting");
  display.display();

  if (!tempsensor.begin(0x1B))
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
  
  accelgyro.initialize();

  tempsensor.setResolution(3);
  
  // Initialize the sensor
  // Failures are generally due to an improper I2C wiring, missing power supply
  // or wrong target chip
  // Set up the wanted parameters
   
  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                  CHARACTERISTIC_UUID_TX,
                  BLECharacteristic::PROPERTY_NOTIFY
                );
  
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  
  Serial.println("Waiting a client connection to notify...");

  Serial.print("Initializing MAX30100..");
  if(!sensor.begin())
    Serial.println("FAILED");
  else Serial.println("SUCCESS");
  sensor.setMode(MAX30100_MODE_SPO2_HR);
  sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
  sensor.setLedsPulseWidth(PULSE_WIDTH);
  sensor.setSamplingRate(SAMPLING_RATE);
  sensor.setHighresModeEnabled(HIGHRES_MODE);
}

int count = 0 , rcount = 0 , modes = 0;
const int dlen = 30 , rlen = 30;
uint8_t d[dlen*4];
long long unsigned int irAC = 0 , redAC = 0;
long long int rbut = 0;

void loop() {
    uint16_t ir, red;
    if(digitalRead(14) == 1)
      rbut = millis();
    if(millis() - rbut > 1000)
    { 
      modes++;
      if(modes > 2)
        modes = 0;
      rbut = millis();
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
      switch(modes)
      {
        case(0):display.drawString(display.getWidth() / 2, display.getHeight() / 2, "SpO2");break;
        case(1):display.drawString(display.getWidth() / 2, display.getHeight() / 2, "Gyro");break;
        case(2):display.drawString(display.getWidth() / 2, display.getHeight() / 2, "Temp");break;
      }
      display.display();
      delay(1000);
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      if (!sensor.begin())
        Serial.println("FAILED");
      else  Serial.println("SUCCESS");
      sensor.setMode(MAX30100_MODE_SPO2_HR);
      sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
      sensor.setLedsPulseWidth(PULSE_WIDTH);
      sensor.setSamplingRate(SAMPLING_RATE);
      sensor.setHighresModeEnabled(HIGHRES_MODE);
    }
    sensor.update();
    while (sensor.getRawValues(&ir, &red)) 
    {
      irAC += ir;
      redAC += red;
      d[4*count] = red>>8;
      d[4*count+1] = red&0xff;
      d[4*count+2] = ir>>8;
      d[4*count+3] = ir&0xff;
      count++;
      rcount++;
      if(count >= dlen)
        count = 0;
      if(count == 0)
      {
        if (deviceConnected) {
            pTxCharacteristic->setValue(d, 4*dlen);
            pTxCharacteristic->notify();
            delay(10); // bluetooth stack will go into congestion, if too many packets are sent
        }
      }
      if(modes == 0 && count == 0)
      {
        for(int i = 0 ; i < dlen ; i++)
          {
            int r = d[4*i] << 8 | d[4*i + 1];
            int rd = d[4*i + 2] << 8 | d[4*i + 3];
            int redl = int(redAC/rcount);
            int irl = int(irAC/rcount);
            Serial.print(r);
            Serial.print('\t');
            Serial.print(rd);
            Serial.print('\t');
            Serial.print(redl);
            Serial.print('\t');
            Serial.print(irl);
            Serial.println('\t');
          }
        int redl = int(redAC/rcount);
        int irl = int(irAC/rcount);
        double ans = 100 * double(redAC/rcount) / double(irAC/rcount);
        if(irl < 1000 || redl < 1000)
          ans = 0;
        display.clear();
        display.drawString(0 , 0 , String("Red : "  +  var2str(redl)));
        display.drawString(0 , 21, String("IR : "   +  var2str(irl)));
        display.drawString(0 , 42, String("SpO2 : " +  var2str(ans , 2)));
        display.display();
      }
      if(rcount >= rlen)
      {
        irAC = 0;
        redAC = 0;
        rcount = 0;
      }
    }
    if(modes == 1)           //accel + grav
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Serial.print(double(ax)/16384); Serial.print("\t");
        Serial.print(double(ay)/16384); Serial.print("\t");
        Serial.print(double(az)/16384); Serial.print("\t");
        Serial.print(double(gx)/16384); Serial.print("\t");
        Serial.print(double(gy)/16384); Serial.print("\t");
        Serial.println(double(gz)/16384);
        display.clear();
        display.drawString(0  , 0 , String("ax:" +  var2str(double(ax)/16384 , 2)));
        display.drawString(0  , 21, String("ay:" +  var2str(double(ay)/16384 , 2)));
        display.drawString(0  , 42, String("az:" +  var2str(double(az)/16384 , 2)));
        display.drawString(64 , 0 , String("gx:" +  var2str(double(gx)/16384 , 2)));
        display.drawString(64 , 21, String("gy:" +  var2str(double(gy)/16384 , 2)));
        display.drawString(64 , 42, String("gz:" +  var2str(double(gz)/16384 , 2)));
        display.display();
    }
    else if(modes == 2)             //temps
    {
      //tempsensor.wake();
      double c = tempsensor.readTempC();
      Serial.print("Temp: "); 
      Serial.print(c, 4); Serial.print("*C\t and ");
      display.clear();
      display.drawString(0  , 0 , String("Temp: " +  var2str(c , 2)));
      display.display();
      //tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
      delay(30);
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        Serial.print("Initializing MAX30100..");
        if (!sensor.begin()) {
            Serial.println("FAILED");
            for(;;);
        } else {
            Serial.println("SUCCESS");
        }
        sensor.setMode(MAX30100_MODE_SPO2_HR);
        sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
        sensor.setLedsPulseWidth(PULSE_WIDTH);
        sensor.setSamplingRate(SAMPLING_RATE);
        sensor.setHighresModeEnabled(HIGHRES_MODE);
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
