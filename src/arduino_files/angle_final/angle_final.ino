#include<ros.h>
#include<std_msgs/Int32MultiArray.h>

#define ENCA_m1 18     // Yellow
#define ENCB_m1 26     // White
int motorPin1_m1 = 3;  // IN1
int motorPin2_m1 = 2;
#define ENCA_m2 19     // Yellow
#define ENCB_m2 28     // White
int motorPin1_m2 = 5;  // IN1
int motorPin2_m2 = 4;
#define ENCA_m3 20     // Yellow
#define ENCB_m3 30     // White
int motorPin1_m3 = 7;  // IN1
int motorPin2_m3 = 6;
#define ENCA_m4 21     // Yellow
#define ENCB_m4 32     // White
int motorPin1_m4 = 9;  // IN1
int motorPin2_m4 = 8;
double recieved = 0;
double target = 0;

ros::NodeHandle nh;

void message_func(const std_msgs::Int32MultiArray& msg){
  if (msg.data_length == 1) {
    recieved = msg.data[0];
    target = abs(recieved);
//    received_value_m1 = msg.data[0];
//    target_m1 = abs(received_value_m1);
//    received_value_m2 = msg.data[1];
//    target_m2 = abs(received_value_m2);
//    received_value_m3 = msg.data[2];
//    target_m3 = abs(received_value_m3);
//    received_value_m4 = msg.data[3];
//    target_m4 = abs(received_value_m4);
  }
}
ros::Subscriber<std_msgs::Int32MultiArray> sub("uc_angs",&message_func);


//int target_joy = 90;
//int target = abs(target_joy);

float kp_m1 = 7;
float kd_m1 = 0.000025;
float ki_m1 = 0.0;
//int pos_m1 = 0;
volatile int posi_m1 = 0;
long prevT_m1 = 0;
float eprev_m1 = 0;
float eintegral_m1 = 0;

float kp_m2 = 7;
float kd_m2 = 0.000025;
float ki_m2 = 0.0;
int pos_m2 = 0;
volatile int posi_m2 = 0;
long prevT_m2 = 0;
float eprev_m2 = 0;
float eintegral_m2 = 0;

float kp_m3 = 7;
float kd_m3 = 0.000025;
float ki_m3 = 0.0;
volatile int posi_m3 = 0;
long prevT_m3 = 0;
float eprev_m3 = 0;
float eintegral_m3 = 0;

float kp_m4 = 7;
float kd_m4 = 0.000025;
float ki_m4 = 0.0;
volatile int posi_m4 = 0;
long prevT_m4 = 0;
float eprev_m4 = 0;
float eintegral_m4 = 0;

void readEncoder_m1() {
  int b_m1 = digitalRead(ENCB_m1);
  if (b_m1 < 0) {
    posi_m1--;
  } else {
    posi_m1++;
  }
}
void readEncoder_m2() {
  int b_m2 = digitalRead(ENCB_m2);
  if (b_m2 < 0) {
    posi_m2--;
  } else {
    posi_m2++;
  }
}
void readEncoder_m3() {
  int b_m3 = digitalRead(ENCB_m3);
  if (b_m3 < 0) {
    posi_m3--;
  } else {
    posi_m3++;
  }
}
void readEncoder_m4() {
  int b_m4 = digitalRead(ENCB_m4);
  if (b_m4 < 0) {
    posi_m4--;
  } else {
    posi_m4++;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCA_m1, INPUT);
  pinMode(ENCB_m1, INPUT);
  
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(ENCA_m2, INPUT);
  pinMode(ENCB_m2, INPUT);
  
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(ENCA_m3, INPUT);
  pinMode(ENCB_m3, INPUT);
  
  pinMode(motorPin1_m3, OUTPUT);
  pinMode(motorPin2_m3, OUTPUT);
  pinMode(ENCA_m4, INPUT);
  pinMode(ENCB_m4, INPUT);
  
  pinMode(motorPin1_m4, OUTPUT);
  pinMode(motorPin2_m4, OUTPUT);
  if (recieved > 0) {
    analogWrite(motorPin2_m1, 255);
    analogWrite(motorPin2_m2, 255);
    analogWrite(motorPin2_m3, 255);
    analogWrite(motorPin2_m4, 255);
  } else {
    analogWrite(motorPin1_m1, 255);
    analogWrite(motorPin1_m2, 255);
    analogWrite(motorPin1_m3, 255);
    analogWrite(motorPin1_m4, 255);
  }
//  else{
//    analogWrite(motorPin2_m1, 0);
//    analogWrite(motorPin2_m2, 0);
//    analogWrite(motorPin2_m3, 0);
//    analogWrite(motorPin2_m4, 0);
//    analogWrite(motorPin1_m1, 0);
//    analogWrite(motorPin1_m2, 0);
//    analogWrite(motorPin1_m3, 0);
//    analogWrite(motorPin1_m4, 0);}

  attachInterrupt(digitalPinToInterrupt(ENCA_m1), readEncoder_m1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_m2), readEncoder_m2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_m3), readEncoder_m3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_m4), readEncoder_m4, RISING);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  long currT_m1 = micros();
  float deltaT_m1 = ((float)(currT_m1 - prevT_m1)) / (1.0e6);
  prevT_m1 = currT_m1;

  int pos_m1 = 0;
  pos_m1 = posi_m1;
  int e_m1 = target - pos_m1;
  float dedt_m1 = (e_m1 - eprev_m1) / (deltaT_m1);
  eintegral_m1 = eintegral_m1 + e_m1 * deltaT_m1;
  float u_m1 = kp_m1 * e_m1 + kd_m1 * dedt_m1 + ki_m1 * eintegral_m1;
  float pwr_m1 = fabs(u_m1);
  if (pwr_m1 > 255) {
    pwr_m1 = 255;
  }
  eprev_m1 = e_m1;
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos_m1);
  Serial.println();
  
  long currT_m2 = micros();
  float deltaT_m2 = ((float)(currT_m2 - prevT_m2)) / (1.0e6);
  prevT_m2 = currT_m2;
    pos_m2 = posi_m2;
  int e_m2 = target - pos_m2;
  float dedt_m2 = (e_m2 - eprev_m2) / (deltaT_m2);
  eintegral_m2 = eintegral_m2 + e_m2 * deltaT_m2;
  float u_m2 = kp_m2 * e_m2 + kd_m2 * dedt_m2 + ki_m2 * eintegral_m2;
  float pwr_m2 = fabs(u_m2);
  if (pwr_m2 > 255) {
    pwr_m2 = 255;
  }

  eprev_m2 = e_m2;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos_m2);
  Serial.println();

  long currT_m3 = micros();
  float deltaT_m3 = ((float)(currT_m3 - prevT_m3)) / (1.0e6);
  prevT_m3 = currT_m3;
  int pos_m3 = 0;
    pos_m3 = posi_m3;
  int e_m3 = target - pos_m3;
  float dedt_m3 = (e_m3 - eprev_m1) / (deltaT_m3);
  eintegral_m3 = eintegral_m3 + e_m3 * deltaT_m3;
  float u_m3 = kp_m1 * e_m3 + kd_m3 * dedt_m3 + ki_m3 * eintegral_m3;
  float pwr_m3 = fabs(u_m3);
  if (pwr_m3 > 255) {
    pwr_m3 = 255;
  }

  eprev_m3 = e_m3;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos_m3);
  Serial.println();

  long currT_m4 = micros();
  float deltaT_m4 = ((float)(currT_m4 - prevT_m4)) / (1.0e6);
  prevT_m4 = currT_m4;
  int pos_m4 = 0;
    pos_m4 = posi_m4;
  int e_m4 = target - pos_m4;
  float dedt_m4 = (e_m4 - eprev_m4) / (deltaT_m4);
  eintegral_m4 = eintegral_m4 + e_m4 * deltaT_m4;
  float u_m4 = kp_m4 * e_m4 + kd_m4 * dedt_m4 + ki_m4 * eintegral_m4;
  float pwr_m4 = fabs(u_m4);
  if (pwr_m4 > 255) {
    pwr_m4 = 255;
  }
  eprev_m4 = e_m4;
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos_m4);
  Serial.println();
  if (recieved > 0) {
    analogWrite(motorPin2_m1, pwr_m1);
    analogWrite(motorPin2_m2, pwr_m2);
    analogWrite(motorPin2_m3, pwr_m3);
    analogWrite(motorPin2_m4, pwr_m4);

  }
  else {
    analogWrite(motorPin1_m1, pwr_m1);
    analogWrite(motorPin1_m2, pwr_m2);
    analogWrite(motorPin1_m3, pwr_m3);
    analogWrite(motorPin1_m4, pwr_m4);
    }

//    else {
//    analogWrite(motorPin2_m1, 0);
//    analogWrite(motorPin2_m2, 0);
//    analogWrite(motorPin2_m3, 0);
//    analogWrite(motorPin2_m4, 0);
//    analogWrite(motorPin1_m1, 0);
//    analogWrite(motorPin1_m2, 0);
//    analogWrite(motorPin1_m3, 0);
//    analogWrite(motorPin1_m4, 0);}

    nh.spinOnce();
}
