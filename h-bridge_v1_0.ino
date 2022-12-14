/*
 H-Bridge control of Mosfets
  This code is modified from public domain Arduino Tutorials
    https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
    SerialToSerialBT by Evandro Copercini, a tutorial built in to our esp32 package

This version of the program controls a single low side mosfet for turning the motor on and off
*/
  int sensorTarget = 0;         //DAC voltage value
  int mosfetsEnabled = 'i';     //tells which mosfets are enabled
  int ctrlMode = 'v';           // int representation of control mode
  int tempValue = 0;
  int trash = 0;
  float targetCurrent = 26;      //target current value from Raspberry Pi
  float targetVoltage = 0;        //target voltage value from Raspberry Pi

#include <driver/dac.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

// initialize pins for transistor control
  pinMode(21, OUTPUT); //Q1 (top left)
  //pinMode(19, OUTPUT); //Q2 (top right)      //not used because ebike will not reverse
  pinMode(18, OUTPUT); //Q3 (bottom left)
  pinMode(5, OUTPUT);  //Q4 (bottom right)     
  pinMode(25, OUTPUT); //DAC target voltage
  pinMode(26, OUTPUT); //DAC mock sensor voltage
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);

//initialize bluetooth communication. Serial communication used for debugging.
  Serial.begin(115200);
  SerialBT.begin("MotorControllerTest");
  //Serial.println("The device started, now you can pair it with bluetooth!");

  //output initialization
  control('i');
  dac_output_voltage(DAC_CHANNEL_1, (int)floor(1.65*256/3.3));  //initial target 0 amps
}

////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // voltage comparison is now done in hardware due to sampling rate limitations. See comparator circuit on github for info.

  // get message from bluetooth
   if (SerialBT.available()){
    ctrlMode = SerialBT.read();            //ctrlMode is either (v)oltage or (c)urrent
    if(ctrlMode == 'v'){
        BT_VoltageTarget();
      } else if (ctrlMode == 'c'){
        BT_CurrentTarget();
      } else {
        BT_StackEmpty();
        Serial.println("Message Recieved");
      }
    
    //briefly enable idle to ensure no undefined behavior
    control('i');
    delayMicroseconds(2);

    // write target current in terms of voltage, then output on DAC pin
    if(targetCurrent > 26){
      targetCurrent = 26;
      }else if(targetCurrent < -26){
        targetCurrent = -26;
      }
    sensorTarget =  (int)(floor((1.65 + targetCurrent*0.055) * 256/3.3));
    dac_output_voltage(DAC_CHANNEL_1, sensorTarget);
    Serial.println(sensorTarget);
    }

//pwm based on voltage target and mosfetsEnabled
if(mosfetsEnabled == 'i'){
  control('i');
}else if(mosfetsEnabled == 'b'){
  control('b');
}else if(mosfetsEnabled == 'f'){
  control('f');
  delayMicroseconds((int)floor(100*targetVoltage));
  control('i');
  delayMicroseconds((int)floor(100*(1-targetVoltage)));
}

}

void BT_StackEmpty()
{
  while(SerialBT.available()){
   trash =  SerialBT.read();
  Serial.println("took out the trash");
  }
  return;
}

void BT_VoltageTarget() //convert percentage to decimal
{
  if(SerialBT.read() == '-'){
    mosfetsEnabled = 'i';
  }else{mosfetsEnabled = 'f';}
  targetVoltage  = ((float)SerialBT.read() - 48)/10;
  targetVoltage += ((float)SerialBT.read() - 48)/100;
  trash = SerialBT.read();                           //begone decimal
  targetVoltage += ((float)SerialBT.read() - 48)/1000;
  BT_StackEmpty();
  Serial.println("target voltage");
  Serial.println(targetVoltage);
  return;
}

void BT_CurrentTarget()
{
  if(SerialBT.read() == '+'){
    mosfetsEnabled = 'f';
    tempValue = 1;
  }else{
    mosfetsEnabled = 'b';
    tempValue = -1;}
  targetCurrent  = ((float)SerialBT.read() - 48)*10;
  targetCurrent += ((float)SerialBT.read() - 48);
  trash = SerialBT.read();                          //begone decimal
  targetCurrent += ((float)SerialBT.read() - 48)/10;
  targetCurrent = tempValue*targetCurrent;
  BT_StackEmpty();
  Serial.println("target current");
  Serial.println(targetCurrent);  
  return;
}

void control (int mosfetsEnabled)
{
//Serial.println("Entered control");

//idle aka coasting. best used to prevent short circuits while switching.
//consider full voltage and low current target for virtual idleing.

  if(mosfetsEnabled == 'i'){
  //digitalWrite(19, LOW);
  digitalWrite(18, LOW); 
  digitalWrite(21, LOW);
  digitalWrite(5, LOW);
  }

//braking current mode  
 if(mosfetsEnabled == 'b'){
    digitalWrite(18, HIGH);
    digitalWrite(5, HIGH);  
    digitalWrite(21, LOW);
    //digitalWrite(19, LOW);
 }

//forward current mode
 if(mosfetsEnabled == 'f'){
    digitalWrite(21, HIGH);
    digitalWrite(5, HIGH);
    //digitalWrite(19, LOW);
    digitalWrite(18, LOW); 
 }

//reverse current targeting. not used in our ebike.
 if(mosfetsEnabled == 'r'){
    //digitalWrite(19, HIGH);
    digitalWrite(18, HIGH); 
    digitalWrite(21, LOW);
    digitalWrite(5, LOW);
  } 
  return;

} 
