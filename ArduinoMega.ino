#include <MQUnifiedsensor.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Fingerprint.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define Board "Arduino MEGA"
#define Type "MQ-2"
#define Voltage_Resolution 5
#define ADC_Bit_Resolution 10
#define RatioMQ2CleanAir 9.83
#define Voltage1_Resolution 5
#define Type1 "MQ-5"
#define ADC1_Bit_Resolution 10
#define RatioMQ5CleanAir 6.5
#define DHTTYPE DHT11
#define SOILMOISTURESENSOR_PIN A0
#define MQ2_Pin A1
#define MQ5_PIN A2
#define mySerial Serial2
#define ECHO 18
#define TRIG 19
#define BUZZER_PIN 24
#define BUZZER1_PIN 28
#define DHT_PIN 30
#define RAINSENSOR_PIN 32
#define FIRESENSOR_PIN 34
#define RELAY1_PIN 36
#define RELAY2_PIN 38
#define RELAY3_PIN 40
#define BUZZER1_TONE 400
#define BUZZER1_SHORT_DELAY 500
#define LEDBEDROOM_PIN 23
#define LEDLIVINGROOM_PIN 25
#define LEDKITCHEN_PIN 27
#define LEDSTAIR_PIN 29
#define LEDWARNING_PIN 31
#define BUTTONDOOR_PIN 33
#define BUTTONRELAY2_PIN 35
#define BUTTONRELAY3_PIN 37
#define BUTTONBEDROOM_PIN 39
#define BUTTONLIVINGROOM_PIN 41
#define BUTTONKITCHEN_PIN 43
#define DOOR_CLOSED 0
#define DOOR_OPENED 100

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ2_Pin, Type);
MQUnifiedsensor MQ5(Board, Voltage1_Resolution, ADC1_Bit_Resolution, MQ5_PIN, Type1);
DHT_Unified dht(DHT_PIN, DHTTYPE);
Adafruit_Fingerprint finger = Adafruit_Fingerprint( & mySerial);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo myservo1;
Servo myservo2;

const size_t JSON_BUFFER_SIZE = 512;
char jsonString[JSON_BUFFER_SIZE];
const unsigned long readingInterval = 1000;
const unsigned long transmissionInterval = 3000;
unsigned long lastReadingTime = 0;
unsigned int readingsCounter = 0;
int doorState = DOOR_CLOSED;
int lastDoorButtonState = HIGH;
int previousButtonStateBedroom = HIGH;
int previousButtonStateLivingroom = HIGH;
int previousButtonStateKitchen = HIGH;
int previousButtonStateRelay2 = HIGH;
int previousButtonStateRelay3 = HIGH;
int setTarget;
bool isAutoWateringEnabled = false;
bool isAutoRainEnabled = false;
bool isWateringButtonSent = false;
bool isRainButtonSent = false;
uint8_t id;
uint8_t fingerTemplate[512];

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  MQ2.setRegressionMethod(1);
  MQ2.setA(574.25);
  MQ2.setB(-2.222);
  MQ2.init();
  MQ5.setRegressionMethod(1);
  MQ5.setA(1163.8);
  MQ5.setB(-3.874);
  MQ5.init();
  float calcR0 = 0;
  float calcR1 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ2.update();
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    MQ5.update();
    calcR1 += MQ5.calibrate(RatioMQ5CleanAir);
  }
  MQ2.setR0(calcR0 / 10);
  MQ5.setR0(calcR1 / 10);
  dht.begin();
  pinMode(RAINSENSOR_PIN, INPUT);
  pinMode(FIRESENSOR_PIN, INPUT);
  pinMode(SOILMOISTURESENSOR_PIN, INPUT);
  pinMode(LEDWARNING_PIN, OUTPUT);
  pinMode(LEDSTAIR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUZZER1_PIN, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LEDBEDROOM_PIN, OUTPUT);
  pinMode(LEDLIVINGROOM_PIN, OUTPUT);
  pinMode(LEDKITCHEN_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(BUTTONDOOR_PIN, INPUT_PULLUP);
  pinMode(BUTTONBEDROOM_PIN, INPUT_PULLUP);
  pinMode(BUTTONLIVINGROOM_PIN, INPUT_PULLUP);
  pinMode(BUTTONKITCHEN_PIN, INPUT_PULLUP);
  pinMode(BUTTONRELAY2_PIN, INPUT_PULLUP);
  pinMode(BUTTONRELAY3_PIN, INPUT_PULLUP);
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  digitalWrite(RELAY3_PIN, HIGH);
  myservo1.attach(22);
  myservo2.attach(26);
  myservo1.write(0);
  myservo2.write(0);
  while (!Serial);
  finger.begin(57600);
  if (finger.verifyPassword()) {
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("WELCOME!");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Fingerprint");
    lcd.setCursor(0, 1);
    lcd.print("sensor error!");
    while (1) {
      delay(1);
    }
  }
  finger.getParameters();
}

float readMQ2() {
  MQ2.update();
  float mq2Value = MQ2.readSensor();
  mq2Value = round(mq2Value * 100) / 100.0;
  if (mq2Value > 1000) {
    sendButtonState("Door", 1);
    myservo2.write(DOOR_OPENED);
    static unsigned long previousFireMillis = 0;
    const long fireInterval = 100;
    if (millis() - previousFireMillis >= fireInterval) {
      previousFireMillis = millis();
      if (digitalRead(LEDWARNING_PIN) == HIGH) {
        digitalWrite(LEDWARNING_PIN, LOW);
      } else {
        digitalWrite(LEDWARNING_PIN, HIGH);
      }
      digitalWrite(BUZZER_PIN, HIGH);
    }
    doorState = DOOR_OPENED;
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LEDWARNING_PIN, LOW);
  }
  return mq2Value;
}

float readMQ5() {
  MQ5.update();
  float mq5Value = MQ5.readSensor();
  mq5Value = round(mq5Value * 100) / 100.0;
  if (mq5Value > 1000) {
    sendButtonState("Door", 1);
    myservo2.write(DOOR_OPENED);
    static unsigned long previousFireMillis = 0;
    const long fireInterval = 100;
    if (millis() - previousFireMillis >= fireInterval) {
      previousFireMillis = millis();
      if (digitalRead(LEDWARNING_PIN) == HIGH) {
        digitalWrite(LEDWARNING_PIN, LOW);
      } else {
        digitalWrite(LEDWARNING_PIN, HIGH);
      }
      digitalWrite(BUZZER_PIN, HIGH);
    }
    doorState = DOOR_OPENED;
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LEDWARNING_PIN, LOW);
  }
  return mq5Value;
}

float readSoilMoistureSensor() {
  int readSensor = 0;
  for (int i = 0; i <= 9; i++) {
    readSensor += analogRead(SOILMOISTURESENSOR_PIN);
  }
  float avgSensor = readSensor / 10;
  float virtualPercent = map(avgSensor, 0, 1023, 0, 100);
  float realPercent = 100 - virtualPercent;
  return realPercent;
}

int readDHT(float & temperature, float & humidity) {
  sensors_event_t event;
  dht.temperature().getEvent( & event);
  temperature = event.temperature;
  dht.humidity().getEvent( & event);
  humidity = event.relative_humidity;
}

int readRainSensor() {
  int rainDigital = digitalRead(RAINSENSOR_PIN);
  return rainDigital;
}

int readFireSensor() {
  int fireDigital = digitalRead(FIRESENSOR_PIN);
  if (fireDigital == 0) {
    sendButtonState("Door", 1);
    myservo2.write(DOOR_OPENED);
    static unsigned long previousFireMillis = 0;
    const long fireInterval = 100;
    if (millis() - previousFireMillis >= fireInterval) {
      previousFireMillis = millis();
      if (digitalRead(LEDWARNING_PIN) == HIGH) {
        digitalWrite(LEDWARNING_PIN, LOW);
      } else {
        digitalWrite(LEDWARNING_PIN, HIGH);
      }
      digitalWrite(BUZZER_PIN, HIGH);
    }
    doorState = DOOR_OPENED;
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LEDWARNING_PIN, LOW);
  }
  return fireDigital;
}

int readDistanceSensor() {
  digitalWrite(TRIG, 0);
  delayMicroseconds(2);
  digitalWrite(TRIG, 1);
  delayMicroseconds(5);
  digitalWrite(TRIG, 0);
  unsigned long duration = pulseIn(ECHO, HIGH);
  int distance = int(duration / 2 / 29.412);
  if (distance < 8) {
    digitalWrite(LEDSTAIR_PIN, HIGH);
  } else {
    digitalWrite(LEDSTAIR_PIN, LOW);
  }
  return distance;
}

void devicesControl() {
  int buttonStateBedroom = digitalRead(BUTTONBEDROOM_PIN);
  int buttonStateLivingroom = digitalRead(BUTTONLIVINGROOM_PIN);
  int buttonStateKitchen = digitalRead(BUTTONKITCHEN_PIN);
  int buttonStateRelay2 = digitalRead(BUTTONRELAY2_PIN);
  int buttonStateRelay3 = digitalRead(BUTTONRELAY3_PIN);
  if (buttonStateBedroom == LOW && previousButtonStateBedroom == HIGH) {
    digitalWrite(LEDBEDROOM_PIN, !digitalRead(LEDBEDROOM_PIN));
    sendButtonState("Bedroom", digitalRead(LEDBEDROOM_PIN));
  }
  if (buttonStateLivingroom == LOW && previousButtonStateLivingroom == HIGH) {
    digitalWrite(LEDLIVINGROOM_PIN, !digitalRead(LEDLIVINGROOM_PIN));
    sendButtonState("Livingroom", digitalRead(LEDLIVINGROOM_PIN));
  }
  if (buttonStateKitchen == LOW && previousButtonStateKitchen == HIGH) {
    digitalWrite(LEDKITCHEN_PIN, !digitalRead(LEDKITCHEN_PIN));
    sendButtonState("Kitchen", digitalRead(LEDKITCHEN_PIN));
  }
  if (buttonStateRelay2 == LOW && previousButtonStateRelay2 == HIGH) {
    digitalWrite(RELAY2_PIN, !digitalRead(RELAY2_PIN));
    sendButtonState("Relay2", !digitalRead(RELAY2_PIN));
  }
  if (buttonStateRelay3 == LOW && previousButtonStateRelay3 == HIGH) {
    digitalWrite(RELAY3_PIN, !digitalRead(RELAY3_PIN));
    sendButtonState("Relay3", !digitalRead(RELAY3_PIN));
  }
  previousButtonStateBedroom = buttonStateBedroom;
  previousButtonStateLivingroom = buttonStateLivingroom;
  previousButtonStateKitchen = buttonStateKitchen;
  previousButtonStateRelay2 = buttonStateRelay2;
  previousButtonStateRelay3 = buttonStateRelay3;
}

void updateDeviceState(const char * buttonName, int buttonState) {
  if (strcmp(buttonName, "Bedroom") == 0) {
    digitalWrite(LEDBEDROOM_PIN, buttonState);
  } else if (strcmp(buttonName, "Livingroom") == 0) {
    digitalWrite(LEDLIVINGROOM_PIN, buttonState);
  } else if (strcmp(buttonName, "Kitchen") == 0) {
    digitalWrite(LEDKITCHEN_PIN, buttonState);
  } else if (strcmp(buttonName, "Door") == 0) {
    if (buttonState == 0) {
      myservo2.write(DOOR_CLOSED);
      delay(1000);
      doorState = DOOR_CLOSED;
    } else {
      myservo2.write(DOOR_OPENED);
      delay(1000);
      doorState = DOOR_OPENED;
    }
  } else if (strcmp(buttonName, "Relay2") == 0) {
    digitalWrite(RELAY2_PIN, !buttonState);
  } else if (strcmp(buttonName, "Relay3") == 0) {
    digitalWrite(RELAY3_PIN, !buttonState);
  }
}

void autoWatering() {
  float soilMoisture = readSoilMoistureSensor();
  if (soilMoisture <= setTarget) {
    digitalWrite(RELAY1_PIN, LOW);
    if (!isWateringButtonSent) {
      sendButtonState("Relay1", 1);
      isWateringButtonSent = true;
    }
  } else {
    digitalWrite(RELAY1_PIN, HIGH);
    if (isWateringButtonSent) {
      sendButtonState("Relay1", 0);
      isWateringButtonSent = false;
    }
  }
}

void autoRain() {
  int rainDigital = readRainSensor();
  if (rainDigital == 1) {
    myservo1.write(180);
    if (!isRainButtonSent) {
      sendButtonState("ServoRain", 1);
      isRainButtonSent = true;
    }
  } else if (rainDigital == 0) {
    myservo1.write(0);
    if (isRainButtonSent) {
      sendButtonState("ServoRain", 0);
      isRainButtonSent = false;
    }
  }
}

void buzz(int buzzerPin, int frequency, int duration, int repeat) {
  for (int i = 0; i < repeat; i++) {
    tone(buzzerPin, frequency);
    delay(duration);
    noTone(buzzerPin);
    delay(200);
  }
}

void handleFingerprintMatch() {
  uint8_t matchedID = finger.fingerID;
  sendButtonState("Door", 1);
  downloadFingerprintTemplate(matchedID);
  sendScanFingerprintJSON(matchedID);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Opening the door");
  buzz(BUZZER1_PIN, BUZZER1_TONE, BUZZER1_SHORT_DELAY, 1);
  myservo2.write(DOOR_OPENED);
  delay(1000);
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("WELCOME!");
  doorState = DOOR_OPENED;
}

void handleFingerprintNotFound() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Try Again!");
  buzz(BUZZER1_PIN, BUZZER1_TONE, BUZZER1_SHORT_DELAY, 2);
  myservo2.write(DOOR_CLOSED);
  delay(1000);
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("WELCOME!");
  doorState = DOOR_CLOSED;
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
  case FINGERPRINT_OK:
    break;
  case FINGERPRINT_NOFINGER:
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    return p;
  case FINGERPRINT_IMAGEFAIL:
    return p;
  default:
    return p;
  }
  p = finger.image2Tz();
  switch (p) {
  case FINGERPRINT_OK:
    break;
  case FINGERPRINT_IMAGEMESS:
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    return p;
  case FINGERPRINT_FEATUREFAIL:
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    return p;
  default:
    return p;
  }
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    handleFingerprintMatch();
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    handleFingerprintNotFound();
  } else {
    return p;
  }
  return finger.fingerID;
}

uint8_t downloadFingerprintTemplate(uint16_t id) {
  uint8_t p = finger.loadModel(id);
  switch (p) {
  case FINGERPRINT_OK:
    break;
  case FINGERPRINT_PACKETRECIEVEERR:
    return p;
  default:
    return p;
  }
  p = finger.getModel();
  switch (p) {
  case FINGERPRINT_OK:
    break;
  default:
    return p;
  }
  uint8_t bytesReceived[534];
  memset(bytesReceived, 0xff, 534);
  uint32_t starttime = millis();
  int i = 0;
  while (i < 534 && (millis() - starttime) < 20000) {
    if (mySerial.available()) {
      bytesReceived[i++] = mySerial.read();
    }
  }
  int uindx = 9, index = 0;
  memcpy(fingerTemplate + index, bytesReceived + uindx, 256); // first 256 bytes
  uindx += 256; // skip data
  uindx += 2; // skip checksum
  uindx += 9; // skip next header
  index += 256; // advance pointer
  memcpy(fingerTemplate + index, bytesReceived + uindx, 256); // second 256 bytes
  for (int i = 0; i < 512; ++i) {
    printHex(fingerTemplate[i], 2);
  }
  return p;
}

void printHex(int num, int precision) {
  char tmp[16];
  char format[128];
  sprintf(format, "%%.%dX", precision);
  sprintf(tmp, format, num);
}

uint8_t readNumber(void) {
  uint8_t num = 0;
  while (num == 0) {
    while (!Serial.available());
    StaticJsonDocument < 64 > doc;
    DeserializationError error = deserializeJson(doc, Serial);
    if (error) {
      Serial.println(error.c_str());
      continue;
    }
    num = doc["fingerId"];
  }
  return num;
}

String readFingerName() {
  String fingerName = "";
  while (fingerName.length() == 0) {
    while (!Serial.available());
    StaticJsonDocument < 64 > doc;
    DeserializationError error = deserializeJson(doc, Serial);
    if (error) {
      Serial.println(error.c_str());
      continue;
    }
    fingerName = doc["fingerName"].as < String > ();
  }
  return fingerName;
}

uint8_t getFingerprintEnroll(int id,
  const String & fingerName) {
  int p = -1;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Place your");
  lcd.setCursor(0, 1);
  lcd.print("finger!");
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      return p;
    case FINGERPRINT_IMAGEFAIL:
      return p;
    default:
      return p;
    }
  }
  p = finger.image2Tz(1);
  switch (p) {
  case FINGERPRINT_OK:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Image converted");
    break;
  case FINGERPRINT_IMAGEMESS:
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    return p;
  case FINGERPRINT_FEATUREFAIL:
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    return p;
  default:
    return p;
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Remove finger");
  delay(1000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Place the same");
  lcd.setCursor(0, 1);
  lcd.print("finger again");
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      return p;
    case FINGERPRINT_IMAGEFAIL:
      return p;
    default:
      return p;
    }
  }
  p = finger.image2Tz(2);
  switch (p) {
  case FINGERPRINT_OK:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Image converted");
    break;
  case FINGERPRINT_IMAGEMESS:
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    return p;
  case FINGERPRINT_FEATUREFAIL:
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    return p;
  default:
    return p;
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Creating model...");
  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Prints matched!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    return p;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    return p;
  } else {
    return p;
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Storing...");
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Stored!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    return p;
  } else if (p == FINGERPRINT_BADLOCATION) {
    return p;
  } else if (p == FINGERPRINT_FLASHERR) {
    return p;
  } else {
    return p;
  }
  return FINGERPRINT_OK;
}

void enrollFingerprint() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Type 1 to 127");
  int id = readNumber();
  if (id == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Invalid ID!");
    delay(1000);
    return;
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter finger name:");
  String fingerName = readFingerName();
  lcd.clear();
  lcd.print("Enrolling ID #");
  lcd.print(id);
  int result = getFingerprintEnroll(id, fingerName);
  lcd.clear();
  if (result == FINGERPRINT_OK) {
    lcd.setCursor(4, 0);
    lcd.print("Enrolled!");
    downloadFingerprintTemplate(id);
    sendEnrollFingerprintJSON(id, fingerName);
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Enroll failed!");
  }
  delay(1000);
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("WELCOME!");
}

void deleteFingerprint() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Type 1 to 127");
  int id = readNumber();
  if (id == 0) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Invalid ID!");
    delay(1000);
    return;
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Deleting ID #");
  lcd.print(id);
  uint8_t result = finger.deleteModel(id);
  if (result == FINGERPRINT_OK) {
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("Deleted!");
    delay(1000);
    downloadFingerprintTemplate(id);
    sendDeleteFingerprintJSON(id);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Deletion failed!");
    delay(1000);
  }
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("WELCOME!");
}

void readFingerprint() {
  getFingerprintID();
  delay(50);
  int buttonState = digitalRead(BUTTONDOOR_PIN);
  if (buttonState == LOW && lastDoorButtonState == HIGH) {
    if (doorState == DOOR_OPENED) {
      sendButtonState("Door", 0);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Closing the door");
      myservo2.write(DOOR_CLOSED);
      delay(1000);
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("WELCOME!");
      doorState = DOOR_CLOSED;
    } else {
      sendButtonState("Door", 1);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Opening the door");
      buzz(BUZZER1_PIN, BUZZER1_TONE, BUZZER1_SHORT_DELAY, 1);
      myservo2.write(DOOR_OPENED);
      delay(1000);
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("WELCOME!");
      doorState = DOOR_OPENED;
    }
  }
  lastDoorButtonState = buttonState;
}

void sendSensorData(float mq2Value, float mq5Value, float soilMoisture, float temperature, float humidity, int rainDigital, int fireDigital, int distance) {
  DynamicJsonDocument jsonDoc(JSON_BUFFER_SIZE);
  JsonObject sensorData = jsonDoc.createNestedObject("sensors");
  sensorData["MQ2"] = mq2Value;
  sensorData["MQ5"] = mq5Value;
  sensorData["Soil"] = soilMoisture;
  sensorData["Temp"] = temperature;
  sensorData["Hum"] = humidity;
  sensorData["Rain"] = rainDigital;
  sensorData["Fire"] = fireDigital;
  sensorData["Dis"] = distance;
  serializeJson(jsonDoc, jsonString);
  Serial.println(jsonString);
}

void sendButtonState(const char * buttonName, int buttonState) {
  DynamicJsonDocument jsonDoc(JSON_BUFFER_SIZE);
  JsonObject buttonData = jsonDoc.createNestedObject("buttons");
  buttonData[buttonName] = buttonState;
  serializeJson(jsonDoc, jsonString);
  Serial.println(jsonString);
}

void sendEnrollFingerprintJSON(uint8_t id,
  const String & fingerName) {
  DynamicJsonDocument jsonDoc(JSON_BUFFER_SIZE);
  JsonObject root = jsonDoc.to < JsonObject > ();
  JsonObject fingerObject = root.createNestedObject("finger");
  JsonArray fingerArray = fingerObject.createNestedArray("fingerEnroll");
  JsonObject fingerprintData = fingerArray.createNestedObject();
  fingerprintData["id"] = id;
  fingerprintData["fingerName"] = fingerName;
  JsonArray dataArray = fingerprintData.createNestedArray("data");
  for (int i = 0; i < 512; ++i) {
    dataArray.add(fingerTemplate[i]);
  }
  serializeJson(jsonDoc, jsonString);
  Serial.println(jsonString);
}

void sendScanFingerprintJSON(uint8_t id) {
  DynamicJsonDocument jsonDoc(JSON_BUFFER_SIZE);
  JsonObject root = jsonDoc.to < JsonObject > ();
  JsonObject fingerObject = root.createNestedObject("finger");
  JsonArray fingerArray = fingerObject.createNestedArray("fingerScan");
  JsonObject fingerprintData = fingerArray.createNestedObject();
  fingerprintData["id"] = id;
  serializeJson(jsonDoc, jsonString);
  Serial.println(jsonString);
}

void sendDeleteFingerprintJSON(uint8_t id) {
  DynamicJsonDocument jsonDoc(JSON_BUFFER_SIZE);
  JsonObject root = jsonDoc.to < JsonObject > ();
  JsonObject fingerObject = root.createNestedObject("finger");
  JsonArray fingerArray = fingerObject.createNestedArray("fingerDelete");
  JsonObject fingerprintData = fingerArray.createNestedObject();
  fingerprintData["id"] = id;
  serializeJson(jsonDoc, jsonString);
  Serial.println(jsonString);
}

void readSensors() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastReadingTime >= readingInterval) {
    lastReadingTime = currentMillis;
    readingsCounter++;
    float mq2Value = readMQ2();
    float mq5Value = readMQ5();
    float soilMoisture = readSoilMoistureSensor();
    float temperature, humidity;
    readDHT(temperature, humidity);
    int rainDigital = readRainSensor();
    int fireDigital = readFireSensor();
    int distance = readDistanceSensor();
    if (readingsCounter >= (transmissionInterval / readingInterval)) {
      readingsCounter = 0;
      sendSensorData(mq2Value, mq5Value, soilMoisture, temperature, humidity, rainDigital, fireDigital, distance);
    }
  }
}

void handleSerialInput() {
  if (Serial.available()) {
    Serial.readBytesUntil('\n', jsonString, JSON_BUFFER_SIZE);
    DynamicJsonDocument jsonDoc(JSON_BUFFER_SIZE);
    DeserializationError error = deserializeJson(jsonDoc, jsonString);
    if (error) {
      Serial.println(error.c_str());
    } else if (jsonDoc.containsKey("buttons")) {
      JsonObject buttonsData = jsonDoc["buttons"];
      for (JsonPair button: buttonsData) {
        String buttonName = button.key().c_str();
        int currentButtonState = button.value().as < int > ();
        updateDeviceState(buttonName.c_str(), currentButtonState);
        if (strcmp(buttonName.c_str(), "Relay1") == 0) {
          if (currentButtonState == 1) {
            digitalWrite(RELAY1_PIN, LOW);
          } else {
            digitalWrite(RELAY1_PIN, HIGH);
          }
        } else if (strcmp(buttonName.c_str(), "ServoRain") == 0) {
          if (currentButtonState == 1) {
            myservo1.write(180);
          } else {
            myservo1.write(0);
          }
        }
      }
    } else if (jsonDoc.containsKey("autoMode")) {
      JsonObject autoModeData = jsonDoc["autoMode"];
      if (autoModeData.containsKey("water")) {
        if (autoModeData["water"].containsKey("setTarget")) {
          int newSetTarget = autoModeData["water"]["setTarget"];
          setTarget = newSetTarget;
        }
        int waterStatus = autoModeData["water"]["status"];
        if (waterStatus == 1 && !isAutoWateringEnabled) {
          isAutoWateringEnabled = true;
          isWateringButtonSent = false;
        } else {
          isAutoWateringEnabled = false;
        }
      }
      if (autoModeData.containsKey("rain")) {
        int rainStatus = autoModeData["rain"]["status"];
        if (rainStatus == 1 && !isAutoRainEnabled) {
          isAutoRainEnabled = true;
          isRainButtonSent = false;
        } else {
          isAutoRainEnabled = false;
        }
      }
    } else if (jsonDoc.containsKey("fingerControl")) {
      int controlValue = jsonDoc["fingerControl"];
      if (controlValue == 1) {
        enrollFingerprint();
      } else if (controlValue == 0) {
        deleteFingerprint();
      }
    }
  }
}

void loop() {
  readFingerprint();
  devicesControl();
  readSensors();
  handleSerialInput();
  if (isAutoWateringEnabled) {
    autoWatering();
  }
  if (isAutoRainEnabled) {
    autoRain();
  }
}