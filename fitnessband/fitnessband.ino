#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"

#define SAMPLE_SIZE 2000

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute=0;
int beatAvg;

int window = 4;

int accelX = 1;
int accelY = 1;
int accelZ = 1;
float x, y, z;

int insertionIndexX = 0;
int charInsertionIndexX = 0;
int samplesX[SAMPLE_SIZE];
char charSamplesX[SAMPLE_SIZE];
long lastRepTimeX = 0;
int repCountX = 0;

int vibrationPin = LED_BUILTIN;

long lastVibrationTime = 0;
bool lastVibrationRep = 0;

// Create BLE service. Arduino will be "peripheral device" in BLE terminology.
// Android app will "Central device".
// Characterstics that'll be transmitted are:
// X, Y, Z acceleration from the accelerometer and Heart rate from Heart Rate sensor

BLEService customService("1101");
BLEUnsignedIntCharacteristic customXChar("54f18fd9-1a17-11ea-8230-48d705ced0b9", BLERead | BLENotify);
BLEUnsignedIntCharacteristic customYChar("1282205e-1c5b-11ea-b8e3-48d705ced0b9", BLERead | BLENotify);
BLEUnsignedIntCharacteristic customZChar("9bff54fa-1c5b-11ea-9d57-48d705ced0b9", BLERead | BLENotify);

BLEUnsignedIntCharacteristic customHRChar("7d9c1ee6-1c5b-11ea-9fbd-48d705ced0b9", BLERead | BLENotify);

// Setup the vibration motor pins and start advertising BLE Beacon
void setup() {
  pinMode(vibrationPin ,OUTPUT);
  IMU.begin();
  Serial.begin(9600); 
  while (!Serial);
  if (!BLE.begin()) {
    Serial.println("BLE failed to Initiate");
    delay(500);
    while (1);
  }
  
  BLE.setLocalName("FITNESS BAND");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(customXChar);
  customService.addCharacteristic(customYChar);
  customService.addCharacteristic(customZChar);
  customService.addCharacteristic(customHRChar);
  BLE.addService(customService);
  customXChar.writeValue(accelX);
  customYChar.writeValue(accelY);
  customZChar.writeValue(accelZ);
  customHRChar.writeValue(beatsPerMinute);
  
   if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
      Serial.println("MAX30105 was not found. Please check wiring/power. ");
    }
  
    particleSensor.setup(); //Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
  BLE.advertise();
  
  Serial.println("Bluetooth device is now active, waiting for connections...");
}

void calculateRep(int value, int id) {
  
  if (id == 0) {
          if (insertionIndexX == SAMPLE_SIZE) {
        Serial.println("overflow");
        insertionIndexX = 0;
      }

      if (charInsertionIndexX == SAMPLE_SIZE) {
        charInsertionIndexX = 0;
      }
      
      samplesX[insertionIndexX++] = accelX;

      if (insertionIndexX % window == 0) {
        int count = 0;
        for (int i = insertionIndexX - window; i >= 1 and i < insertionIndexX; ++i) {
          if (samplesX[i - 1] > samplesX[i]) {
            ++count;
          }
        }
        
        if (count >= (window + 1) / 2) {
          charSamplesX[charInsertionIndexX++] = 'u';
        } else {
          charSamplesX[charInsertionIndexX++] = 'd';
        }
      }
      
      bool flag = true;
      for (int i = charInsertionIndexX - (2 * window); i >= 0 and i < charInsertionIndexX; ++i) {

        if (i < charInsertionIndexX - window and charSamplesX[i] != 'u') {
          flag = false;          
        } else if (i >= charInsertionIndexX - window and charSamplesX[i] != 'd') {
          flag = false;
        }

//        Serial.print(charSamples[i]);
      }
//            Serial.println(flag);

      if (flag == true and ((millis() - lastRepTimeX) > 1000)) {
        Serial.println("repcount");
        repCountX++;
        customXChar.writeValue(1);
        lastRepTimeX= millis();
      }
   }
}

void loop() {
  
  digitalWrite(vibrationPin,LOW);
  getHeartRate();
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    while (central.connected()) {
      read_Accel();
      delay(50);
      calculateRep(accelX, 0);

      if (repCountX % 5 == 0 and lastVibrationRep != repCountX) {
        Serial.println("rep yes");
        vibrate(true);
        lastVibrationRep = repCountX;
        lastVibrationTime = millis();
      }

      if (millis() - lastVibrationTime > 500) {
         vibrate(false);
      }

      Serial.print(accelX);
      Serial.print(" ");
      Serial.print(accelY);
      Serial.print(" ");
      Serial.print(accelZ);
      Serial.println();
    }
  }
  
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
  
}

void getHeartRate() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  
  customHRChar.writeValue(beatsPerMinute);
}

void read_Accel() {

  if (IMU.accelerationAvailable()) {
  IMU.readAcceleration(x, y, z);
  accelX = (x + 1) * 100;
  accelY = (y + 1) * 100;
  accelZ = (z + 1) * 100;
  }
}

void vibrate(bool yes){
//  if (yes)
//    digitalWrite(vibrationPin,HIGH);
//  else 
//    digitalWrite(vibrationPin,LOW);
}
