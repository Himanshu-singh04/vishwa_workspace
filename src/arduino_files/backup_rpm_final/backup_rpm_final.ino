#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

int motorPin1_m1 = 2;
int motorPin2_m1 = 3;

int motorPin1_m2 = 4;
int motorPin2_m2 = 5;

int motorPin1_m3 = 6;
int motorPin2_m3 = 7;

int motorPin1_m4 = 8;
int motorPin2_m4 = 9;

//int dir = 150;
double pwr_m1 = 0;
double received_value_m1 = 0;
double pwr_m2 = 0;
double received_value_m2 = 0;
double pwr_m3 = 0;
double received_value_m3 = 0;
double pwr_m4 = 0;
double received_value_m4 = 0;

ros::NodeHandle nh;

void uc_velsCallback(const std_msgs::Int32MultiArray& msg) {
  if (msg.data_length == 4) {
    received_value_m1 = msg.data[0];
    pwr_m1 = abs(received_value_m1);
    received_value_m2 = msg.data[1];
    pwr_m2 = abs(received_value_m2);
    received_value_m3 = msg.data[2];
    pwr_m3 = abs(received_value_m3);
    received_value_m4 = msg.data[3];
    pwr_m4 = abs(received_value_m4);
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("wheel_vels", &uc_velsCallback);

void setup(){
  Serial.begin(115200);
  
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);

  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);

  pinMode(motorPin1_m3, OUTPUT);
  pinMode(motorPin2_m3, OUTPUT);

  pinMode(motorPin1_m4, OUTPUT);
  pinMode(motorPin2_m4, OUTPUT);

  if (received_value_m1 > 0){analogWrite(motorPin2_m1, 255);}
  else {analogWrite(motorPin1_m1, 255);}  

  if (received_value_m2 > 0){analogWrite(motorPin2_m2, 255);}
  else {analogWrite(motorPin1_m2, 255);}  

  if (received_value_m3 > 0){analogWrite(motorPin2_m3, 255);}
  else {analogWrite(motorPin1_m3, 255);}  

  if (received_value_m4 > 0){analogWrite(motorPin2_m4, 255);}
  else {analogWrite(motorPin1_m4, 255);} 

  nh.initNode();
  nh.subscribe(sub);
}
  
void loop(){
  if (received_value_m1 > 0){analogWrite(motorPin2_m1, pwr_m1);}
  else {analogWrite(motorPin1_m1, pwr_m1);}

  if (received_value_m2 > 0){analogWrite(motorPin2_m2, pwr_m2);}
  else {analogWrite(motorPin1_m2, pwr_m2);}

  if (received_value_m3 > 0){analogWrite(motorPin2_m3, pwr_m3);}
  else {analogWrite(motorPin1_m3, pwr_m3);}

  if (received_value_m4 > 0){analogWrite(motorPin2_m4, pwr_m4);}
  else {analogWrite(motorPin1_m4, pwr_m4);}

  nh.spinOnce();
  }
