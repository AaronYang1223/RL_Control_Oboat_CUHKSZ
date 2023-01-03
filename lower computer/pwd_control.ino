#include <Servo.h>
#include<PS2X_lib.h>

Servo ESC_R, ESC_L;

PS2X psx;
void setup() {
 Serial.begin(9600);
 psx.config_gamepad(A3,A1,A2,A0,false,false); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
 ESC_R.attach(5, 1000, 2000);
 ESC_L.attach(6, 1000, 2000);
 ESC_R.write(90);
 ESC_L.write(90); 
 delay(5000);
}

void loop() {
 psx.read_gamepad();
 if (psx.Button(PSB_PAD_RIGHT)) {
   ESC_R.write(150);
   ESC_L.write(30);
 }
 else if (psx.Button(PSB_PAD_LEFT)) {
   ESC_R.write(30);
   ESC_L.write(150);
 }
 else if (psx.Button(PSB_PAD_UP)) {
   ESC_R.write(150);
   ESC_L.write(150);
 } 
 else if (psx.Button(PSB_PAD_DOWN)) {
   ESC_R.write(30);
   ESC_L.write(30);
 }
 
 else {
   Serial.println("Nothing...");
   ESC_R.write(90);
   ESC_L.write(90);
 }
 delay(50);
}
