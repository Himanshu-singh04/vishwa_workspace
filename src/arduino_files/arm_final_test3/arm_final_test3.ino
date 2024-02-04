#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

int base_pin1 = 44;
int base_pin2 = 46;
int link1_Pin1 = 11;
int link1_Pin2 = 10;
int link2_Pin1 = 9;
int link2_Pin2 = 8;

int bevel1_Pin1 = 3;
int bevel1_Pin2 = 2;
int bevel2_Pin1 = 5;
int bevel2_Pin2 = 4;
int claw_Pin1 = 7;
int claw_Pin2 = 6;

double pwr_m1 = 0;
double received_value_m1 = 0;
double pwr_m2 = 0;
double received_value_m2 = 0;
double pwr_m3 = 0;
double received_value_m3 = 0;

double pwr_m4 = 0;
double received_value_m4 = 0;
double pwr_m5 = 0;
double received_value_m5 = 0;
double pwr_m6 = 0;
double received_value_m6 = 0;

ros::NodeHandle nh;

void uc_velsCallback(const std_msgs::Int32MultiArray& msg) {
   if (msg.data_length == 6) {
     received_value_m1 = msg.data[0];
     pwr_m1 = received_value_m1;
     
     received_value_m2 = msg.data[1];
     pwr_m2 = received_value_m2;
     
     received_value_m3 = msg.data[2];
     pwr_m3 = received_value_m3;
     
     received_value_m4 = msg.data[3];
     pwr_m4 = received_value_m4;
     
     received_value_m5 = msg.data[4];
     pwr_m5 = received_value_m5;
     
     received_value_m6 = msg.data[5];
     pwr_m6 = received_value_m6;
   }
 }

ros::Subscriber<std_msgs::Int32MultiArray> sub("arm", &uc_velsCallback);

void setup() {
  Serial.begin(115200);

  pinMode(base_pin1, OUTPUT);
  pinMode(base_pin2, OUTPUT);

  pinMode(link1_Pin1, OUTPUT);
  pinMode(link1_Pin2, OUTPUT);

  pinMode(link2_Pin1, OUTPUT);
  pinMode(link2_Pin2, OUTPUT);

  pinMode(bevel1_Pin1, OUTPUT);
  pinMode(bevel1_Pin2, OUTPUT);

  pinMode(bevel2_Pin1, OUTPUT);
  pinMode(bevel2_Pin2, OUTPUT);

  pinMode(claw_Pin1, OUTPUT);
  pinMode(claw_Pin2, OUTPUT);

//  if (received_value_m1 > 0) {
//    analogWrite(base_pin1, 255);
//    analogWrite(base_pin2, 0);
//  } else {
//    analogWrite(base_pin1, 0);
//    analogWrite(base_pin2, 255);
//  }
//
//  if (received_value_m2 > 0) {
//   analogWrite(link1_Pin1, 255);
//   analogWrite(link1_Pin2, 0);
//  } else {
//   analogWrite(link1_Pin1, 0);
//   analogWrite(link1_Pin2, 255);
//  }
//
//  if (received_value_m3 > 0) {
//     analogWrite(link2_Pin1, 255);
//     analogWrite(link2_Pin2, 0);
//   } else {
//     analogWrite(link2_Pin1, 0);
//     analogWrite(link2_Pin2, 255);
//   }
//
//   if (received_value_m4 > 0) {
//     analogWrite(bevel1_Pin1, 255);
//     analogWrite(bevel1_Pin2, 0);
//   } else {
//     analogWrite(bevel1_Pin1, 0);
//     analogWrite(bevel1_Pin2, 255);
//   }
//   if (received_value_m5 > 0) {
//     analogWrite(bevel2_Pin1, 255);
//     analogWrite(bevel2_Pin2, 0);
//   } else {
//     analogWrite(bevel2_Pin1, 0);
//     analogWrite(bevel2_Pin2, 255);
//   }
//   if (received_value_m6 > 0) {
//     analogWrite(claw_Pin1, 255);
//     analogWrite(claw_Pin2, 0);
//   } else {
//     analogWrite(claw_Pin1, 0);
//     analogWrite(claw_Pin2, 255);
//   }
   nh.initNode();
   nh.subscribe(sub);
}

void loop() {
   if (pwr_m1 > 0) {
     analogWrite(base_pin1, abs(pwr_m1));
     analogWrite(base_pin2, 0);
   } else {
     analogWrite(base_pin1, 0);
     analogWrite(base_pin2, abs(pwr_m1));
   }

   if (pwr_m2 > 0) {
    analogWrite(link1_Pin1, abs(pwr_m2));
    analogWrite(link1_Pin2, 0);
    
   } else {
     analogWrite(link1_Pin1, 0);
     analogWrite(link1_Pin2, abs(pwr_m2));

   }

  if (pwr_m3 > 0) {
     analogWrite(link2_Pin1, abs(pwr_m3));
     analogWrite(link2_Pin2, 0);
     
   } else {
    analogWrite(link2_Pin1, 0);
    analogWrite(link2_Pin2, abs(pwr_m3));

   }

  if (pwr_m4 > 0) {
    analogWrite(bevel1_Pin2, abs(pwr_m4));
    analogWrite(bevel1_Pin1, 0);
  } else {analogWrite(bevel1_Pin2, 0);
  analogWrite(bevel1_Pin1, abs(pwr_m4));
  }

  if (pwr_m5 > 0) {
    analogWrite(bevel2_Pin1, abs(pwr_m5));
    analogWrite(bevel2_Pin2, 0);
  } else {analogWrite(bevel2_Pin1, 0);
  analogWrite(bevel2_Pin2, abs(pwr_m5));
  }

  if (pwr_m6 > 0) {
    analogWrite(claw_Pin1, abs(pwr_m6));
    analogWrite(claw_Pin2, 0);
  } else {analogWrite(claw_Pin1, 0);
  analogWrite(claw_Pin2, abs(pwr_m6));
  }

   nh.spinOnce();
}
