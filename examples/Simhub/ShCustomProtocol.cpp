#ifndef __SHCUSTOMPROTOCOL_H__
#define __SHCUSTOMPROTOCOL_H__

#include <Arduino.h>
#include <VolvoDIM.h>

// Create an instance of VolvoDIM for the protocol
VolvoDIM VolvoDIM(9, 6);

class SHCustomProtocol {
public:
  void setup() {
    VolvoDIM.gaugeReset();
    VolvoDIM.init();
  }
  
  void read() {
    // Read gauge values from serial input (turn-signal logic removed)
    int coolantTemp = FlowSerialReadStringUntil(',').toInt();
    int carSpeed = FlowSerialReadStringUntil(',').toInt();
    int rpms = FlowSerialReadStringUntil(',').toInt();
    int fuelPercent = FlowSerialReadStringUntil(',').toInt();
    int oilTemp = FlowSerialReadStringUntil(',').toInt();
    String gear = FlowSerialReadStringUntil(',');
    int hour = FlowSerialReadStringUntil(',').toInt();
    int minute = FlowSerialReadStringUntil(',').toInt();
    int mileage = FlowSerialReadStringUntil(',').toInt();
    int ding = FlowSerialReadStringUntil(',').toInt();
    int totalBrightness = FlowSerialReadStringUntil(',').toInt();
    int highbeam = FlowSerialReadStringUntil(',').toInt();
    int fog = FlowSerialReadStringUntil(',').toInt();
    int brake = FlowSerialReadStringUntil(',').toInt();
    int rightblinker = FlowSerialReadStringUntil(',').toInt();
    int leftblinker  = FlowSerialReadStringUntil(',').toInt(); 
    int hazard = FlowSerialReadStringUntil(',').toInt(); 
    int parkingBrake = FlowSerialReadStringUntil(',').toInt();
    String text = FlowSerialReadStringUntil(','); 
    int service = FlowSerialReadStringUntil('\n'); 
    
    int timeValue = VolvoDIM.clockToDecimal(hour, minute, 1);
    
    VolvoDIM.setTime(timeValue);
    VolvoDIM.setOutdoorTemp(oilTemp);
    VolvoDIM.setCoolantTemp(coolantTemp);
    VolvoDIM.setSpeed(carSpeed);
    VolvoDIM.setGasLevel(fuelPercent);
    VolvoDIM.setRpm(rpms);
    VolvoDIM.setGearPosText(gear.c_str());
    VolvoDIM.enableMilageTracking(mileage);
    VolvoDIM.enableDisableDingNoise(ding);
    VolvoDIM.enableHighBeam(highbeam);
    VolvoDIM.setTotalBrightness(totalBrightness);
    VolvoDIM.enableFog(fog);
    VolvoDIM.enableBrake(brake);
    VolvoDIM.setBlinker(rightblinker, leftblinker, hazard);
    VolvoDIM.enableParkingBrake(parkingBrake);
    VolvoDIM.displayText(text.c_str());
    VolvoDIM.clearServiceMessage(service);

  }
  
  void loop() {
    VolvoDIM.simulate();
  }
  
  void idle() {}
};

#endif
