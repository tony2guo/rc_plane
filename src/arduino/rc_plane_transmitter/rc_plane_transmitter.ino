/* 
 *Author: tony2guo
 *Modification of DIY Arduino based RC Transmitter by Dejan Nedelkovski, www.HowToMechatronics.com
 *Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>

RF24 radio(A1, A0);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001"; // Address

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  int8_t esc = -127;
  int8_t servo1;
  int8_t servo2;
  int8_t servo3;
  int8_t servo4;
};

Data_Package data; //Create a variable with the above structure

void rc_plane_cmdCB(const std_msgs::Int8MultiArray& msg)
{
  data.esc=msg.data[0];
  data.servo1=msg.data[1];
  data.servo2=msg.data[2];
  data.servo3=msg.data[3];
  data.servo4=msg.data[4];
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int8MultiArray> rc_cmd_sub("rc_plane_cmd_throttle", rc_plane_cmdCB);

void setup() {
  //Serial.begin(115200); //rosserial can't use this with arduino pro mini, but ok with Leonardo?
  
  // Define the radio communication
  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS);//RF24_250KBPS RF24_1MBPS RF24_2MBPS
  radio.setPALevel(RF24_PA_MAX);//RF24_PA_MIN RF24_PA_LOW RF24_PA_HIGH RF24_PA_MAX

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(rc_cmd_sub);
}
void loop() {
  // Send the whole data from the structure to the receiver
//  data.esc=random(256);
//  data.servo1=random(256);
//  data.servo2=random(256);
//  data.servo3=random(256);
//  data.servo4=random(256);
  radio.write(&data, sizeof(Data_Package));
  nh.spinOnce();
  //delay(50);
}
