/*
 H-Bridge control of LEDs


  This code is modified from public domain Arduino Tutorials
    https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
    SerialToSerialBT by Evandro Copercini, a tutorial built in to our esp32 package
    
  additional credit to Skeletron, whose code I used in lines 66, 67 to format the string
  https://stackoverflow.com/questions/67780199/receiving-string-data-via-bluetooth-esp32?newreg=976c4fdb5f89405a849debf09fd41fdd
*/

  int sensorValue = 0;            // value read from the potentiometer
  float outputValue = 0;          // calculated current draw
  int ctrlMode = 105;             // int representation of control mode
  float tempValue = 0;    
  float ctrlValue = 0;            //target current value
  String message = "";            //holds incoming bluetooth message


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
// pin for voltage reading
  pinMode(32, INPUT);

// initialize pins for transistor control
  pinMode(21, OUTPUT); //Q1 (top left)
  pinMode(19, OUTPUT); //Q2 (top right)
  pinMode(18, OUTPUT); //Q3 (bottom left)
  pinMode(5, OUTPUT);  //Q4 (bottom right)

//initialize bluetooth communication
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // we should probably use interrupts and timers for the analog reading later to ensure fast operation.
  // i believe it is easy to run interrupt events on a separate core because it is an "event".
  // the compiler has an option to automatically thread events separately. more research required
  
  // read the analog in value
  sensorValue = analogRead(32);
  // map it to the range of the current
  outputValue = float(sensorValue - 2048)/2048*30; // calculates amps. 0 (0) volts is -30 amps. 3.3 (4097) volts is +30 amps
  Serial.print(outputValue);
  Serial.print("\n");


  // get message from bluetooth
   if (SerialBT.available()){
    ctrlMode = SerialBT.read();                           //ctrlMode is either (f)orward, (r)everse, (i)dle, (b)rake
    message = SerialBT.readStringUntil('\n');
    message.remove(message.length()-1, 1);                //removes empty characters from bluetooth message
    tempValue  = ((float)message.charAt(0) - 48)*10;
    tempValue += ((float)message.charAt(1) - 48);
    tempValue += ((float)message.charAt(3) - 48)/10;
    tempValue += ((float)message.charAt(4) - 48)/100;
    ctrlValue  = tempValue;
    }
  
  

 //pin write logic

//idle aka coasting
  if(ctrlMode == 'i'){
  digitalWrite(19, LOW);
  digitalWrite(18, LOW); 
  digitalWrite(21, LOW);
  digitalWrite(5, LOW);
  }

//braking torque current target  
 if(ctrlMode == 'b'){
  if(outputValue > -ctrlValue && outputValue < ctrlValue){
    digitalWrite(18, HIGH);
    digitalWrite(5, HIGH);  
    digitalWrite(21, LOW);
    digitalWrite(19, LOW);
  } 
  else{
    digitalWrite(19, LOW);
    digitalWrite(18, LOW); 
    digitalWrite(21, LOW);
    digitalWrite(5, LOW);  
  }
 }

//forward torque current target
 if(ctrlMode == 'f'){
  if(outputValue < ctrlValue){
    digitalWrite(21, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(19, LOW);
    digitalWrite(18, LOW); 
  }
  else{
    digitalWrite(19, LOW);
    digitalWrite(18, LOW); 
    digitalWrite(21, LOW);
    digitalWrite(5, LOW);     
  }
 }

//reverse torque current targeting 
 if(ctrlMode == 'r'){
  if(outputValue > -ctrlValue){
    digitalWrite(19, HIGH);
    digitalWrite(18, HIGH); 
    digitalWrite(21, LOW);
    digitalWrite(5, LOW);
  } 
  else{
    digitalWrite(19, LOW);
    digitalWrite(18, LOW); 
    digitalWrite(21, LOW);
    digitalWrite(5, LOW);     
  }
 }

 delay(50);
   
}
