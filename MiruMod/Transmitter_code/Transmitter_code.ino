#include <SPI.h>
  #include <nRF24L01.h>
  #include <RF24.h>
   
  RF24 radio(7, 8); // select CE,CSN pin 
  const byte address[6] = "00001";
  struct Signal {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
  byte aux1;
  byte aux2;
};
  Signal data;
  
  void ResetData() 
{
  data.throttle = 127;   // Default position of motors
  data.pitch = 127;    
  data.roll = 127;     
  data.yaw = 127;     
  data.aux1 = 127;    
  data.aux2 = 127;    
}
  void setup()
{
  Serial.begin(9600);
  //Start everything up
  radio.begin();
  radio.openWritingPipe(address);
  radio.stopListening(); //start the radio comunication for Transmitter 
 
}
  // Joystick center and its borders 
  int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
  val = map(val, lower, middle, 0, 128);
  else
  val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}
  void loop()
{
  ResetData();
  // Control Stick Calibration 
  // Setting may be required for the correct values of the control levers. 
  data.throttle = mapJoystickValues( analogRead(A0), 12, 524, 1020, true );  // "true" or "false" for signal direction 
  data.roll = mapJoystickValues( analogRead(A3), 12, 524, 1020, true );      // "true" or "false" for servo direction 
  data.pitch = mapJoystickValues( analogRead(A2), 12, 524, 1020, true );     // "true" or "false" for servo direction 
  data.yaw = mapJoystickValues( analogRead(A1), 12, 524, 1020, true );       // "true" or "false" for servo direction 
  data.aux1 = mapJoystickValues( analogRead(A4), 12, 524, 1020, true );     // "true" or "false" for servo direction 
  data.aux2 = mapJoystickValues( analogRead(A5), 12, 524, 1020, true );// "true" or "false" for servo direction 
  Serial.print(data.throttle);
  Serial.print('\t');
  Serial.print(data.roll);
  Serial.print('\t');
  Serial.print(data.pitch);
  Serial.print('\t');
  Serial.print(data.yaw);
  Serial.print('\t');
  Serial.print(data.aux1);
  Serial.print('\t');
  Serial.print(data.aux2);
  Serial.println('\t');
  
  radio.write(&data, sizeof(Signal));
}
