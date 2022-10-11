/*
 H-Bridge control of Mosfets
  This code is modified from public domain Arduino Tutorials
    https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
    SerialToSerialBT by Evandro Copercini, a tutorial built in to our esp32 package

This version of the program controls a single low side mosfet for turning the motor on and off
*/
  int targetVoltage = 0;        //DAC voltage value
  int mockSensorValue = 0;      //temporary DAC voltage value for 
  int ctrlMode = 'i';           // int representation of control mode
  float tempValue = 0;
  float targetCurrent = 0;      //target current value from Raspberry Pi    
  String message = "";          //holds incoming bluetooth message

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
  pinMode(5, OUTPUT);  //Q4 (bottom right)     //single transistor for this experiment
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
  dac_output_voltage(DAC_CHANNEL_1, (int)floor(1.65*256/3.3));
  dac_output_voltage(DAC_CHANNEL_2, (int)floor(2.2*256/3.3));
}

////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // voltage comparison is now done in hardware due to sampling rate limitations. See comparator circuit on github for info.

  // get message from bluetooth
   if (SerialBT.available()){
    ctrlMode = SerialBT.read();                           //ctrlMode is either (f)orward, (r)everse, (i)dle, (b)rake
    message = SerialBT.readStringUntil('\n');
    message.remove(message.length()-1, 1);                //removes empty characters from bluetooth message
    targetCurrent  = ((float)message.charAt(0) - 48)*10;
    targetCurrent += ((float)message.charAt(1) - 48);
    targetCurrent += ((float)message.charAt(3) - 48)/10;
    targetCurrent += ((float)message.charAt(4) - 48)/100;
    Serial.println(targetCurrent);
    
    //briefly enable idle to ensure no undefined behavior
    control('i');
    delayMicroseconds(2);

    // write target current in terms of voltage, then output on DAC pin
    if(targetCurrent > 26){targetCurrent = 26;}
    if(ctrlMode == 'b'){targetCurrent = -1*targetCurrent;}
    targetVoltage =  (int)(floor((1.65 + targetCurrent*0.055) * 256/3.3));
    Serial.println(targetVoltage);
    dac_output_voltage(DAC_CHANNEL_1, targetVoltage);

    control(ctrlMode);
    }

  
}

void control (int ctrlMode)
{
//idle aka coasting. best used to prevent short circuits while switching.
//Drive 00.00 or Brake 00.00 will be better for passive coast.

  if(ctrlMode == 'i'){
  //digitalWrite(19, LOW);
  digitalWrite(18, LOW); 
  digitalWrite(21, LOW);
  digitalWrite(5, LOW);
  }

//braking current target  
 if(ctrlMode == 'b'){
    digitalWrite(18, HIGH);
    digitalWrite(5, HIGH);  
    digitalWrite(21, LOW);
    //digitalWrite(19, LOW);
 }

//forward current target
 if(ctrlMode == 'f'){
    digitalWrite(21, HIGH);
    digitalWrite(5, HIGH);
    //digitalWrite(19, LOW);
    digitalWrite(18, LOW); 
 }

//reverse current targeting. not used in our ebike.
 if(ctrlMode == 'r'){
    //digitalWrite(19, HIGH);
    digitalWrite(18, HIGH); 
    digitalWrite(21, LOW);
    digitalWrite(5, LOW);
  } 
  return;

} 
