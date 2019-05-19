/* 
 *Author: tony2guo
 *Modification of DIY Arduino based RC Transmitter by Dejan Nedelkovski, www.HowToMechatronics.com
 *Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(A1, A0);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001";

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

Servo esc;  // create servo object to control the ESC
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
int escValue, servo1Value, servo2Value;

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte esc;
  byte servo1;
  byte servo2;
  byte servo3;
  byte servo4;
};

Data_Package data; //Create a variable with the above structure

void setup() {
  //Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver
  resetData();
  esc.attach(0);
  servo1.attach(4);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(7);
}
void loop() {
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  // Controlling servos
  //servo1Value = map(data.servo1, 0, 255, 0, 180);
  //servo2Value = map(data.servo2, 0, 255, 0, 180);
  servo1.write(map(data.servo1, 0, 255, 0, 180));
  servo2.write(map(data.servo2, 0, 255, 0, 180));
  servo3.write(map(data.servo3, 0, 255, 0, 180));
  servo4.write(map(data.servo4, 0, 255, 0, 180));
  // Controlling brushless motor with ESC
  //escValue = map(data.esc, 0, 255, 1000, 2000); // Map the receiving value form 0 to 255 to 0 1000 to 2000, values used for controlling ESCs
  esc.writeMicroseconds(map(data.esc, 0, 255, 1000, 2000)); // Send the PWM control singal to the ESC
//  Serial.print(data.esc);
//  Serial.print(" ");
//  Serial.print(data.servo1);
//  Serial.print(" ");
//  Serial.print(data.servo2);
//  Serial.print(" ");
//  Serial.print(data.servo3);
//  Serial.print(" ");
//  Serial.print(data.servo4);
//  Serial.print(" \n");
}

void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.esc = 0;
  data.servo1 = 90;
  data.servo2 = 90;
  data.servo3 = 90;
  data.servo4 = 90;
}
