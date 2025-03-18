/*
  VolvoDIM.cpp - Library for powering and controlling a P2 Volvo DIM.
  Created by Andrew J. Gabler, August 17, 2021.
  Adapted by Flybrick_S60R, March 12, 2025.
*/
#include "VolvoDIM.h"
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#endif

// Create a CAN object using the provided CS pin.
mcp2515_can CAN(0);
int _relayPin = 0;

VolvoDIM::VolvoDIM(int SPI_CS_PIN, int relayPin) {
  mcp2515_can temp_CAN(SPI_CS_PIN);
  CAN = temp_CAN;
  _relayPin = relayPin;
  
  // Initialize parking brake relay control pin (digital pin 7)
  _parkingBrakePin = 7;
  pinMode(_parkingBrakePin, OUTPUT);
  // Set default state: relay off (assuming active low for parking brake)
  digitalWrite(_parkingBrakePin, HIGH);
}

// Global variables and constants
bool enableSerialErrMsg = false;
int genCnt = 0;
int cnt = 0;
constexpr int listLen = 14;
char* customTextMessage = "";
int startUpWait = 0;
int customMessageCnt = 0, customTextChanged = 0;
int mileageCounter = 0, mileagePace = 0, genSpeed = 0;
int mileageEnabled = 1;
#define CALIBRATION_FACTOR 830.0  // Final calibration factor for the odometer takes 1min 56 seconds 70 ms to cover 1 mile at 64mph.
unsigned long lastUpdateTimeGlobal = 0;
float mileageAccumulatorGlobal = 0.0;
int carConCnt = 0;
int configCnt = 0;
int blinkerInterval = 0;
unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long address;


// Array indices for defaultData
constexpr int arrSpeed    = 0;  // Speed/KeepAlive, CAN ID: 0x217FFC
constexpr int arrRpm      = 1;  // RPM/Backlights, CAN ID: 0x2803008
constexpr int arrCoolant  = 2;  // Coolant/OutdoorTemp, CAN ID: 0x3C01428
constexpr int arrTime     = 3;  // Time/GasTank (time and fuel), CAN ID: 0x381526C
constexpr int arrBrakes   = 4;  // Brake system keep alive, CAN ID: 0x3600008
constexpr int arrBlinker  = 5;  // Blinker, CAN ID: 0xA10408
constexpr int arrAntiSkid = 6;  // Anti-Skid, CAN ID: 0x2006428
constexpr int arrAirbag   = 7;  // Airbag Light, CAN ID: 0x1A0600A
constexpr int arr4c       = 8;  // 4C keep alive, CAN ID: 0x2616CFC
constexpr int arrConfig   = 9;  // Car Config, CAN ID: 0x1017FFC
constexpr int arrGear     = 10; // Gear Position, CAN ID: 0x3200408
constexpr int arrDmWindow = 11; // Dim Message Window, CAN ID: 0x02A0240E
constexpr int arrDmMessage= 12; // Dim Message Content, CAN ID: 0x1800008
constexpr int arrDisplay  = 13; // Display Rotate OEM, CAN ID: 0x0131726C

// Address list for CAN messages.
constexpr unsigned long addrLi[listLen] = {
  0x217FFC, 0x2803008, 0x3C01428, 0x381526C, 0x3600008,
  0xA10408, 0x2006428, 0x1A0600A, 0x2616CFC, 0x1017FFC,
  0x3200408, 0x02A0240E, 0x1800008, 0x131726C
};

// Default data for each message slot.
unsigned char defaultData[listLen][8] = {
  {0x01, 0xEB, 0x00, 0xD8, 0xF0, 0x58, 0x00, 0x00}, // 0: Speed/KeepAlive
  {0xFF, 0xE1, 0xFF, 0xFF, 0xFF, 0xCF, 0x00, 0x00}, // 1: RPM/Backlights
  {0xC0, 0x80, 0x51, 0x89, 0x0E, 0x57, 0x00, 0x00}, // 2: Coolant/OutdoorTemp
  {0x00, 0x01, 0x05, 0xBC, 0x05, 0xA0, 0x40, 0x40}, // 3: Time/GasTank
  {0x00, 0x00, 0xB0, 0x60, 0x30, 0x00, 0x00, 0x00}, // 4: Brake system keep alive
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08}, // 5: Blinker
  {0x01, 0xE3, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00}, // 6: Anti-Skid
  {0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x49, 0x00}, // 7: Airbag Light
  {0x0B, 0x42, 0x00, 0x00, 0xFD, 0x1F, 0x00, 0xFF}, // 8: 4C keep alive
  {0x01, 0x0F, 0xF7, 0xFA, 0x00, 0x00, 0x00, 0xC0}, // 9: Car Config
  {0x11, 0xDE, 0x53, 0x00, 0x24, 0x00, 0x10, 0x00}, // 10: Gear Position
  {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35}, // 11: Dim Message Window
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00}, // 12: Dim Message Content
  {0xCF, 0xEB, 0x80, 0xA2, 0xF0, 0xAA, 0x00, 0xAA}  // 13: Display Rotate OEM
};

// ---------------------- Message Transmission Functions ----------------------

void VolvoDIM::sendMsgWrapper(unsigned long wId, unsigned char *wBuf)
{
  CAN.sendMsgBuf(wId, 1, 8, wBuf);
}

void VolvoDIM::initSRS()
{
  unsigned char temp[8] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0xBC, 0xDB, 0x80};
  unsigned long tempAddr = addrLi[arrAirbag];
  sendMsgWrapper(tempAddr, temp);
  delay(15);
  temp[0] = 0x00;
  sendMsgWrapper(tempAddr, temp);
  delay(15);
  temp[0] = 0xC0;
  temp[6] = 0xC9;
  sendMsgWrapper(tempAddr, temp);
  delay(15);
  temp[0] = 0x80;
  sendMsgWrapper(tempAddr, temp);
  delay(15);
}

void VolvoDIM::genSRS(long address, byte stmp[])
{
  int randomNum = random(0, 794);
  if (randomNum < 180) {
    stmp[0] = 0x40;
  } else if (randomNum < 370) {
    stmp[0] = 0xC0;
  } else if (randomNum < 575) {
    stmp[0] = 0x00;
  } else {
    stmp[0] = 0x80;
  }
  sendMsgWrapper(address, stmp);
  delay(15);
}

void VolvoDIM::init4C()
{
  unsigned char temp[8] = {0x09, 042, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00};
  unsigned long tempAddr = addrLi[arr4c];
  sendMsgWrapper(tempAddr, temp);
  delay(15);
  sendMsgWrapper(tempAddr, temp);
  delay(15);
  temp[0] = 0x0B;
  sendMsgWrapper(tempAddr, temp);
  delay(15);
  sendMsgWrapper(tempAddr, temp);
  delay(15);
}

void VolvoDIM::genCC(long address, byte stmp[])
{
  int randomNum = random(0, 7);
  if (randomNum > 3) {
    stmp[6] = 0xFF;
    stmp[7] = 0xF3;
  }
  sendMsgWrapper(address, stmp);
  delay(15);
}

void VolvoDIM::genTemp(long address, byte stmp[])
{
  int randomNum = random(0, 133);
  if (randomNum < 16) {
    stmp[0] = 0x00;
  } else if (randomNum < 41) {
    stmp[0] = 0x40;
  } else if (randomNum < 76) {
    stmp[0] = 0xC0;
  } else {
    stmp[0] = 0x80;
  }
  stmp[1] = (randomNum < 76) ? 0x80 : 0x00;
  randomNum = random(0, 105);
  if (randomNum < 7) {
    stmp[2] = 0x11;
  } else if (randomNum < 15) {
    stmp[2] = 0x71;
  } else if (randomNum < 25) {
    stmp[2] = 0x61;
  } else if (randomNum < 60) {
    stmp[2] = 0x51;
  } else {
    stmp[2] = 0x41;
  }
  sendMsgWrapper(address, stmp);
  delay(15);
}

void VolvoDIM::genMileageAndSpeed() {
    unsigned long currentTime = millis();
    if (lastUpdateTimeGlobal == 0) {
        lastUpdateTimeGlobal = currentTime;
    }
    unsigned long deltaMillis = currentTime - lastUpdateTimeGlobal;
    lastUpdateTimeGlobal = currentTime;
    
    if (mileageEnabled == 1 && startUpWait == 0) {
        float deltaHours = deltaMillis / 3600000.0;
        float distanceIncrement = genSpeed * deltaHours;
        
        // Use the calibration factor to match the odometer
        mileageAccumulatorGlobal += distanceIncrement * CALIBRATION_FACTOR;
        
        int rawUnitsToAdd = (int)mileageAccumulatorGlobal;
        if (rawUnitsToAdd > 0) {
            mileageCounter += rawUnitsToAdd;
            mileageAccumulatorGlobal -= rawUnitsToAdd;
        }
        if (mileageCounter > 255) {
            mileageCounter = 0;
        }
        defaultData[arrSpeed][7] = mileageCounter;
    }
    sendMsgWrapper(addrLi[arrSpeed], defaultData[arrSpeed]);
}

void VolvoDIM::powerOn()
{
    pinMode(_relayPin, OUTPUT);
    digitalWrite(_relayPin, HIGH);
}

void VolvoDIM::powerOff()
{
    pinMode(_relayPin, OUTPUT);
    digitalWrite(_relayPin, LOW);
}

void VolvoDIM::gaugeReset()
{
    powerOn();
    delay(7000);
    powerOff();
}

void VolvoDIM::init()
{
    while (CAN_OK != CAN.begin(CAN_125KBPS, MCP_16MHz))
    {
        delay(100);
    }
    if (_relayPin > 0)
    {
        powerOn();
    }
    initSRS();
    init4C();
}

void VolvoDIM::setTime(int inputTime)
{
  if (inputTime >= 0 && inputTime <= 1440) {
    int b4 = 0;
    for (int i = 0; i < 6; i++) {
      if (inputTime >= (i * 256) && inputTime < ((i + 1) * 256)) {
        b4 = i;
      }
    }
    int b5 = inputTime - (b4 * 256);
    defaultData[arrTime][4] = b4;
    defaultData[arrTime][5] = b5;
  }
}

int VolvoDIM::clockToDecimal(int hour, int minute, int AM)
{
  if ((hour >= 0 && hour <= 12) && (minute >= 0 && minute < 60) && (AM == 1 || AM == 0)) {
    if (AM)
      return (hour == 12) ? minute : ((hour * 60) + minute);
    return ((hour * 60) + minute) + 720;
  }
  return 0;
}

double VolvoDIM::celsToFahr(double temp)
{
  return ((temp * (9.0 / 5.0)) + 32);
}

void VolvoDIM::setOutdoorTemp(int oTemp)
{
  if (!(oTemp < -49 || oTemp > 176)) {
    if (oTemp <= 32) {
      defaultData[arrCoolant][4] = 0x0D;
      defaultData[arrCoolant][5] = ceil((oTemp + 83) * 2.21);
    } else if (oTemp <= 146) {
      defaultData[arrCoolant][4] = 0x0E;
      defaultData[arrCoolant][5] = ceil((oTemp - 33) * 2.25);
    } else if (oTemp < 177) {
      defaultData[arrCoolant][4] = 0x0F;
      defaultData[arrCoolant][5] = ceil((oTemp - 147) * 2.20);
    }
  }
}

void VolvoDIM::setCoolantTemp(int range)
{
  if (range >= 0 && range <= 55) {
    defaultData[arrCoolant][3] = range + 88;
  } else if (range <= 100) {
    defaultData[arrCoolant][3] = ceil((range - 55) * 0.333) + 173;
  }
}

void VolvoDIM::setSpeed(int carSpeed)
{
  genSpeed = carSpeed;
  if (carSpeed >= 0 && carSpeed <= 160) {
    if (carSpeed <= 40)
      defaultData[arrSpeed][5] = 0x58;
    else if (carSpeed <= 80)
      defaultData[arrSpeed][5] = 0x59;
    else if (carSpeed <= 120)
      defaultData[arrSpeed][5] = 0x5A;
    else
      defaultData[arrSpeed][5] = 0x5B;
    defaultData[arrSpeed][6] = round(carSpeed * 6.375);
  }
}

void VolvoDIM::setGasLevel(int level)
{
  if (level >= 0 && level <= 100)
  {
    int value = round(level * 0.62);
    defaultData[arrTime][6] = value;
    defaultData[arrTime][7] = value;
    sendMsgWrapper(addrLi[arrTime], defaultData[arrTime]);
  }
  else
  {
    if (enableSerialErrMsg)
    {
      // Serial.println("Gas level out of range");
    }
  }
}

void VolvoDIM::setRpm(int rpm) {
  // Clamp rpm between 0 and 8000
  if (rpm < 0)
    rpm = 0;
  if (rpm > 8000)
    rpm = 8000;
  
  // Adjusted full-scale constant: 8000 rpm should map to ~31.62 (instead of 24)
  const float scale = 31.62 / 8000.0;
  float fixed_point = rpm * scale;  // This gives a value from 0.0 up to ~31.62
  
  // Convert to a Q8.8 fixed–point value
  uint16_t encoded = (uint16_t)round(fixed_point * 256.0);
  
  // Byte 6 holds the high byte (the "major" part)
  defaultData[arrRpm][6] = encoded >> 8;
  // Byte 7 holds the low byte (the "minor" adjustments)
  defaultData[arrRpm][7] = encoded & 0xFF;
}


void VolvoDIM::enableHighBeam(int enabled) {
  if (enabled == 1)
    defaultData[arrRpm][1] = 0xFF;
  else
    defaultData[arrRpm][1] = 0xEA;
}

void VolvoDIM::setTotalBrightness(int value)
{
    if (value < 0)
        value = 0;
    else if (value > 255)
        value = 255;
    defaultData[arrRpm][2] = value;
    int byte3 = 0x30 + static_cast<int>((value * 15.0f / 255.0f) + 0.5f);
    defaultData[arrRpm][3] = byte3;
    int byte4 = 0x30 + static_cast<int>((value * 13.0f / 255.0f) + 0.5f);
    defaultData[arrRpm][4] = byte4;
}

void VolvoDIM::setGearPosText(const char* gear)
{
  if (gear[0] == 'l' || gear[0] == 'L') {
    defaultData[arrGear][4] = 0x40;
    defaultData[arrGear][6] = 0x96;
  } else if (gear[0] == 'p' || gear[0] == 'P') {
    defaultData[arrGear][4] = 0x24;
    defaultData[arrGear][6] = 0x10;
  } else if (gear[0] == 'r' || gear[0] == 'R') {
    defaultData[arrGear][4] = 0xE4;
    defaultData[arrGear][6] = 0x20;
  } else if (gear[0] == 'n' || gear[0] == 'N') {
    defaultData[arrGear][4] = 0x24;
    defaultData[arrGear][6] = 0x30;
  } else if (gear[0] == '1') {
    defaultData[arrGear][4] = 0x34;
    defaultData[arrGear][6] = 0x40;
  } else if (gear[0] == '2') {
    defaultData[arrGear][4] = 0x40;
    defaultData[arrGear][6] = 0x70;
  } else if (gear[0] == '3') {
    defaultData[arrGear][4] = 0x40;
    defaultData[arrGear][6] = 0x60;
  } else if (gear[0] == '4') {
    defaultData[arrGear][4] = 0x44;
    defaultData[arrGear][6] = 0x50;
  } else if (gear[0] == '5') {
    defaultData[arrGear][4] = 0xBE;
    defaultData[arrGear][6] = 0x00;
  } else if (gear[0] == '6') {
    defaultData[arrGear][4] = 0xD5;
    defaultData[arrGear][6] = 0x00;
  } else {
    defaultData[arrGear][4] = 0x10;
    defaultData[arrGear][6] = 0x40;
  }
}

void VolvoDIM::setGearPosInt(int gear)
{
  switch (gear) {
    case -4:
      defaultData[arrGear][4] = 0x40; defaultData[arrGear][6] = 0x96; break;
    case -3:
      defaultData[arrGear][4] = 0x24; defaultData[arrGear][6] = 0x10; break;
    case -2:
      defaultData[arrGear][4] = 0xE4; defaultData[arrGear][6] = 0x20; break;
    case -1:
      defaultData[arrGear][4] = 0x10; defaultData[arrGear][6] = 0x40; break;
    case 0:
      defaultData[arrGear][4] = 0x24; defaultData[arrGear][6] = 0x30; break;
    case 1:
      defaultData[arrGear][4] = 0x34; defaultData[arrGear][6] = 0x40; break;
    case 2:
      defaultData[arrGear][4] = 0x40; defaultData[arrGear][6] = 0x70; break;
    case 3:
      defaultData[arrGear][4] = 0x40; defaultData[arrGear][6] = 0x60; break;
    case 4:
      defaultData[arrGear][4] = 0x44; defaultData[arrGear][6] = 0x50; break;
    case 5:
      defaultData[arrGear][4] = 0xBE; defaultData[arrGear][6] = 0x00; break;
    case 6:
      defaultData[arrGear][4] = 0xD5; defaultData[arrGear][6] = 0x00; break;
  }
}

// ------------------ Custom Text Display Functions ------------------
//
// This helper function formats the input text into a 32‑character string
// corresponding to two lines of 16 characters each. It does simple word wrapping
// so that words are moved to the next line if they would exceed 16 characters.
String formatTextForDisplay(const char* text) {
  String input = text;
  input.trim();
  
  String line1 = "";
  String line2 = "";
  
  int start = 0;
  while (start < input.length()) {
    int spaceIdx = input.indexOf(' ', start);
    String word;
    if (spaceIdx == -1) {
      word = input.substring(start);
      start = input.length();
    } else {
      word = input.substring(start, spaceIdx);
      start = spaceIdx + 1;
    }
    word.trim();
    if (word.length() == 0)
      continue;
      
    // If line1 is empty, add the word (truncate if necessary)
    if (line1.length() == 0) {
      if (word.length() > 16)
        word = word.substring(0, 16);
      line1 = word;
    } else {
      // Try to add with a preceding space
      if (line1.length() + 1 + word.length() <= 16) {
        line1 += " " + word;
      } else {
        // Word does not fit in line1, so add to line2 (if line2 empty, truncate if too long)
        if (line2.length() == 0) {
          if (word.length() > 16)
            word = word.substring(0, 16);
          line2 = word;
        } else {
          if (line2.length() + 1 + word.length() <= 16)
            line2 += " " + word;
          else
            break; // Only two lines available
        }
      }
    }
  }
  
  while (line1.length() < 16)
    line1 += " ";
  while (line2.length() < 16)
    line2 += " ";
    
  return line1 + line2; // 32-character string (16 characters per line)
}

void VolvoDIM::setCustomText(const char* text) {
  customTextMessage = (char*)text;
  customMessageCnt = 0;
  customTextChanged = 1;
}

void VolvoDIM::genCustomText(const char* text) {
  // Format the text into a 32-character (16x2) message using word wrap.
  String formatted = formatTextForDisplay(text);
  
  // Convert the formatted String to a char array.
  char msg[33];
  formatted.toCharArray(msg, 33);
  
  // Activate the custom text display command.
  memcpy(stmp, defaultData[arrDmWindow], sizeof(stmp));
  defaultData[arrDmWindow][7] = 0x31;
  sendMsgWrapper(addrLi[arrDmWindow], stmp);
  delay(40);
  
  // Send the text via D2 protocol frames.
  // We send the message in chunks.
  const int firstChunk = 6;  // first frame holds 6 characters (after header bytes)
  const int chunkSize = 7;   // subsequent frames carry 7 characters
  stmp[0] = 0xA7;
  stmp[1] = 0x00;
  memcpy(&stmp[2], msg, firstChunk);
  sendMsgWrapper(addrLi[arrDmMessage], stmp);
  delay(40);
  
  int index = firstChunk;
  byte seq = 0x21;
  while (index < 32) {
    stmp[0] = seq;
    int copySize = (32 - index >= chunkSize) ? chunkSize : (32 - index);
    char buffer[chunkSize];
    memset(buffer, ' ', chunkSize);
    memcpy(buffer, &msg[index], copySize);
    memcpy(&stmp[1], buffer, chunkSize);
    sendMsgWrapper(addrLi[arrDmMessage], stmp);
    delay(40);
    index += chunkSize;
    seq++;
  }
  
  // Final frame to complete transmission.
  stmp[0] = 0x65;
  memset(&stmp[1], ' ', 7);
  sendMsgWrapper(addrLi[arrDmMessage], stmp);
}

void VolvoDIM::clearCustomText()
{
  unsigned char clearValues[] = {0xE1, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendMsgWrapper(addrLi[arrDmWindow], clearValues);
}

void VolvoDIM::displayText(const char* text) {
  genCustomText(text);
}

void VolvoDIM::enableMilageTracking(int on){
  mileageEnabled = on;
}

void VolvoDIM::enableDisableDingNoise(int on){
  memcpy(stmp, defaultData[arrTime], sizeof(stmp));
  if(on == 0){
    stmp[1] = 0x30;
    sendMsgWrapper(addrLi[arrTime], stmp);
  } else if (on == 1){
    stmp[1] = 0x18;
    sendMsgWrapper(addrLi[arrTime], stmp);
  }
}

void VolvoDIM::enableFog(int enabled)
{
  if (enabled == 1)
    defaultData[arr4c][2] = 0xE6;
  else
    defaultData[arr4c][2] = 0x00;
}

void VolvoDIM::enableBrake(int enabled)
{
  if (enabled == 1)
    defaultData[arrBrakes][3] = 0x00;
  else
    defaultData[arrBrakes][3] = 0x60;
}

void VolvoDIM::setBlinker(int right, int left, int hazard) {
  // Priority: hazard > right > left
  if (hazard == 1 || (right == 1 && left == 1))
    defaultData[arrBlinker][7] = 0x0E;
  else if (right == 1)
    defaultData[arrBlinker][7] = 0x0C;
  else if (left == 1)
    defaultData[arrBlinker][7] = 0x0A;
  else
    defaultData[arrBlinker][7] = 0x08;
}

void VolvoDIM::enableParkingBrake(int enabled) {
  if (enabled == 1)
    digitalWrite(_parkingBrakePin, HIGH);
  else
    digitalWrite(_parkingBrakePin, LOW);
}

void VolvoDIM::clearServiceMessage(int enabled) {
  if (enabled == 1)
    defaultData[arrDisplay][7] = 0x7F;
  else
    defaultData[arrDisplay][7] = 0x3F;
}

void VolvoDIM::sweepGauges()
{
  setRpm(7900);
  setSpeed(160);
  delay(500);
  setRpm(0);
  setSpeed(0);
}

void VolvoDIM::enableSerialErrorMessages()
{
  enableSerialErrMsg = true;
}

void VolvoDIM::disableSerialErrorMessages()
{
  enableSerialErrMsg = false;
}

// -------------------- Simulation Functions --------------------

void VolvoDIM::simulateHighPriority() {
  genMileageAndSpeed();
  sendMsgWrapper(addrLi[arrRpm], defaultData[arrRpm]);
}

void VolvoDIM::simulateLowPriority() {
  sendMsgWrapper(addrLi[arrBrakes], defaultData[arrBrakes]);
  sendMsgWrapper(addrLi[arrAntiSkid], defaultData[arrAntiSkid]);
  genSRS(addrLi[arrAirbag], defaultData[arrAirbag]);
  sendMsgWrapper(addrLi[arr4c], defaultData[arr4c]);
  genCC(addrLi[arrConfig], defaultData[arrConfig]);
  sendMsgWrapper(addrLi[arrTime], defaultData[arrTime]);
  genTemp(addrLi[arrCoolant], defaultData[arrCoolant]);
  sendMsgWrapper(addrLi[arrBlinker], defaultData[arrBlinker]);
  sendMsgWrapper(addrLi[arrGear], defaultData[arrGear]);
  sendMsgWrapper(addrLi[arrDmWindow], defaultData[arrDmWindow]);
  sendMsgWrapper(addrLi[arrDmMessage], defaultData[arrDmMessage]);
  sendMsgWrapper(addrLi[arrDisplay], defaultData[arrDisplay]);
}

void VolvoDIM::simulate() {
  simulateHighPriority();
  static unsigned long lastLowPriorityTime = 0;
  if (millis() - lastLowPriorityTime >= 50) {
    simulateLowPriority();
    lastLowPriorityTime = millis();
  }
}

void VolvoDIM::sendCANMessage(unsigned long canId, byte data[8]) {
  sendMsgWrapper(canId, data);
}
