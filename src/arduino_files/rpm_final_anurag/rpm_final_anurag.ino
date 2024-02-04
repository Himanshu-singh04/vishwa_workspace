#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <avr/wdt.h>

// rear left
#define ENCA_m1 21
#define ENCB_m1 26 
int motorPin1_m1 = 2; //lpwm
int motorPin2_m1 = 3; //rpwm

// rear right
#define ENCA_m2 19
#define ENCB_m2 28
int motorPin1_m2 = 4; //rpwm
int motorPin2_m2 = 5; //lpwm

// front right
 #define ENCA_m3 20
 #define ENCB_m3 24
 int motorPin2_m3 = 7; //rpqm
 int motorPin1_m3 = 6; //lpwm
 
// front left
 #define ENCA_m4 18
 #define ENCB_m4 30
 int motorPin1_m4 = 8; //rpwm
 int motorPin2_m4 = 9; //lpwm
 
double target_m1 = 0;  // RPM
double received_value_m1 = 0; //dir

double target_m2 = 0;  // RPM
double received_value_m2 = 0; //dir

double target_m3 = 0;  // RPM
double received_value_m3 = 0; //dir

double target_m4 = 0;  // RPM
double received_value_m4 = 0; //dir

ros::NodeHandle nh;

void uc_velsCallback(const std_msgs::Int32MultiArray& msg) {
  if (msg.data_length == 4) {
    
    received_value_m1 = msg.data[0];
//    target_m1 = abs(received_value_m1);
    target_m1 = received_value_m1;
    
    received_value_m2 = msg.data[1];
//    target_m2 = abs(received_value_m2);
    target_m2 = received_value_m2;
        
    received_value_m3 = msg.data[2];
//    target_m3 = abs(received_value_m3);
    target_m3 = received_value_m3;
    
    received_value_m4 = msg.data[3];
//    target_m4 = abs(received_value_m4);
    target_m4 = received_value_m4;
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("wheel_vels", &uc_velsCallback);

unsigned long t_before_m1 = 0;
unsigned long t_after_m1 = 0;
unsigned long t_m1 = 0;
double rpm_m1 = 0;

double e_m1 = 0;
double eprev_m1 = 0;
double eintegral_m1 = 0;
float kp_m1 = 0.5;
float kd_m1 = 0;
float ki_m1 = 0.00001;
long currT_m1 = 0;
double deltaT_m1 = 1;
long prevT_m1 = 0;

unsigned long t_before_m2 = 0;
unsigned long t_after_m2 = 0;
unsigned long t_m2 = 0;
double rpm_m2 = 0;

double e_m2 = 0;
double eprev_m2 = 0;
double eintegral_m2 = 0;
float kp_m2 = 0.5;
float kd_m2 = 0;
float ki_m2 = 0.00001;
long currT_m2 = 0;
double deltaT_m2 = 1;
long prevT_m2 = 0;

unsigned long t_before_m3 = 0;
unsigned long t_after_m3 = 0;
unsigned long t_m3 = 0;
double rpm_m3 = 0;

double e_m3 = 0;
double eprev_m3 = 0;
double eintegral_m3 = 0;
float kp_m3 = 0.5;
float kd_m3 = 0;
float ki_m3 = 0.00001;
long currT_m3 = 0;
double deltaT_m3 = 1;
long prevT_m3 = 0;

unsigned long t_before_m4 = 0;
unsigned long t_after_m4 = 0;
unsigned long t_m4 = 0;
double rpm_m4 = 0;

double e_m4 = 0;
double eprev_m4 = 0;
double eintegral_m4 = 0;
float kp_m4 = 0.5;
float kd_m4 = 0;
float ki_m4 = 0.00001;
long currT_m4 = 0;
double deltaT_m4 = 1;
long prevT_m4 = 0;

void readEncA_m1() {
  int encb_state=digitalRead(ENCB_m1);

  t_before_m1 = t_after_m1;
  t_after_m1 = micros();
  t_m1 = t_after_m1 - t_before_m1;
  rpm_m1 = 181000.0 / t_m1;

  if(1==encb_state)
  {
//    forward
rpm_m1=rpm_m1;
  }

  else if(0==encb_state)
  {
//    backward
    rpm_m1=-rpm_m1;
  }

//  Serial.print("ECNB_m1= ");
//  Serial.println(rpm_m1);
  
}

void readEncA_m2() {
    int encb_state=digitalRead(ENCB_m2);

  t_before_m2 = t_after_m2;
  t_after_m2 = micros();
  t_m2 = t_after_m2 - t_before_m2;
  rpm_m2 = 181000.0 / t_m2;

    if(0==encb_state)
  {
//    forward
rpm_m2=rpm_m2;
  }

  else if(1==encb_state)
  {
//    backward
    rpm_m2=-rpm_m2;
  }

//  Serial.print("ECNB_m2= ");
//  Serial.println(rpm_m2);
  
}

void readEncA_m3() {
  int encb_state=digitalRead(ENCB_m3);

  t_before_m3 = t_after_m3;
  t_after_m3 = micros();
  t_m3 = t_after_m3 - t_before_m3;
  rpm_m3 = 181000.0 / t_m3;

    if(0==encb_state)
  {
//    forward
rpm_m3=rpm_m3;
  }

  else if(1==encb_state)
  {
//    backward
    rpm_m3=-rpm_m3;
  }
//  Serial.print("ECNB_m3= ");
//  Serial.println(rpm_m3);
  
}

void readEncA_m4() {
    int encb_state=digitalRead(ENCB_m4);

  t_before_m4 = t_after_m4;
  t_after_m4 = micros();
  t_m4 = t_after_m4 - t_before_m4;
  rpm_m4 = 181000.0 / t_m4;

    if(1==encb_state)
  {
//    forward
rpm_m4=rpm_m4;
  }

  else if(0==encb_state)
  {
//    backward
    rpm_m4=-rpm_m4;
  }

//  Serial.print("ECNB_m4= ");
///  Serial.println(rpm_m4);
  
}

void setup() {
  Serial.begin(57600);
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
//
  pinMode(ENCA_m4, INPUT);
  pinMode(ENCB_m4, INPUT);
  pinMode(motorPin1_m4, OUTPUT);
  pinMode(motorPin2_m4, OUTPUT);

//  if (received_value_m1 > 0) {
//    analogWrite(motorPin1_m1, 255);
//  } else if (received_value_m1 == 0) {
//    analogWrite(motorPin1_m1, 0);
//    analogWrite(motorPin2_m1, 0);
//  } else if (received_value_m1 < 0) {
//    analogWrite(motorPin2_m1, 255);
//  }

//  if (received_value_m2 > 0) {
//    analogWrite(motorPin1_m2, 255);
//  } else if (received_value_m2 == 0) {
//    analogWrite(motorPin1_m2, 0);
//    analogWrite(motorPin2_m2, 0);
//  } else if (received_value_m2 < 0) {
//    analogWrite(motorPin2_m2, 255);
//  }

//  if (received_value_m3 > 0) {
//    analogWrite(motorPin1_m3, 255);
//  } else if (received_value_m3 == 0) {
//    analogWrite(motorPin1_m3, 0);
//    analogWrite(motorPin2_m3, 0);
//  } else if (received_value_m3 < 0) {
//    analogWrite(motorPin2_m3, 255);
//  }

//  if (received_value_m4 > 0) {
//    analogWrite(motorPin1_m4, 255);
//  } else if (received_value_m4 == 0) {
//    analogWrite(motorPin1_m4, 0);
//    analogWrite(motorPin2_m4, 0);
//  } else if (received_value_m4 < 0) {
//    analogWrite(motorPin2_m4, 255);
//  }
  attachInterrupt(digitalPinToInterrupt(ENCA_m1), readEncA_m1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_m2), readEncA_m2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_m3), readEncA_m3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_m4), readEncA_m4, RISING);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  currT_m1 = micros();
  deltaT_m1 = double((currT_m1 - prevT_m1));
  prevT_m1 = currT_m1;
  e_m1 = target_m1 - rpm_m1;
  eintegral_m1 = eintegral_m1 + e_m1 * deltaT_m1;

  double u_m1 = kp_m1 * e_m1 + ki_m1 * eintegral_m1 + kd_m1 * (rpm_m1 - eprev_m1) / deltaT_m1;
  eprev_m1 = rpm_m1;
//  float pwr_m1 = fabs(u_m1);
//
//  if (pwr_m1 > 255) {
//    pwr_m1 = 255;
//  }
  float pwr_m1 = u_m1;
  if(pwr_m1>255)
  {
    pwr_m1=255;
  }
  else if(pwr_m1<-255)
  {
    pwr_m1=-255;
  }
  

  currT_m2 = micros();
  deltaT_m2 = double((currT_m2 - prevT_m2));
  prevT_m2 = currT_m2;
  e_m2 = target_m2 - rpm_m2;
  eintegral_m2 = eintegral_m2 + e_m2 * deltaT_m2;

  double u_m2 = kp_m2 * e_m2 + ki_m2 * eintegral_m2 + kd_m2 * (rpm_m2 - eprev_m2) / deltaT_m2;
  eprev_m2 = rpm_m2;
//  float pwr_m2 = fabs(u_m2);
//
//  if (pwr_m2 > 255) {
//    pwr_m2 = 255;
//  }
  float pwr_m2 = u_m2;
  if(pwr_m2>255)
  {
    pwr_m2=255;
  }
  else if(pwr_m2<-255)
  {
    pwr_m2=-255;
  }

  currT_m3 = micros();
  deltaT_m3 = double((currT_m3 - prevT_m3));
  prevT_m3 = currT_m3;
  e_m3 = target_m3 - rpm_m3;
  eintegral_m3 = eintegral_m3 + e_m3 * deltaT_m3;

  double u_m3 = kp_m3 * e_m3 + ki_m3 * eintegral_m3 + kd_m3 * (rpm_m3 - eprev_m3) / deltaT_m3;
  eprev_m3 = rpm_m3;
//  float pwr_m3 = fabs(u_m3);
//
//  if (pwr_m3 > 255) {
//    pwr_m3 = 255;
//  }
  float pwr_m3 = u_m3;
  if(pwr_m3>255)
  {
    pwr_m3=255;
  }
  else if(pwr_m3<-255)
  {
    pwr_m3=-255;
  }


  currT_m4 = micros();
  deltaT_m4 = double((currT_m4 - prevT_m4));
  prevT_m4 = currT_m4;
  e_m4 = target_m4 - rpm_m4;
  eintegral_m4 = eintegral_m4 + e_m4 * deltaT_m4;

  double u_m4 = kp_m4 * e_m4 + ki_m4 * eintegral_m4 + kd_m4 * (rpm_m4 - eprev_m4) / deltaT_m4;
  eprev_m4 = rpm_m4;
//  float pwr_m4 = fabs(u_m4);
//
//  if (pwr_m4 > 255) {
//    pwr_m4 = 255;
//  }
  float pwr_m4 = u_m4;
  if(pwr_m4>255)
  {
    pwr_m4=255;
  }
  else if(pwr_m4<-255)
  {
    pwr_m4=-255;
  }


  if (pwr_m1 < 0) {
    analogWrite(motorPin1_m1, -pwr_m1);
    analogWrite(motorPin2_m1, 0);
  } else if (pwr_m1 == 0) {
    analogWrite(motorPin2_m1, 0);
    analogWrite(motorPin1_m1, 0);
  } else if (pwr_m1 > 0) {
    analogWrite(motorPin2_m1, pwr_m1);
    analogWrite(motorPin1_m1, 0);
  }


  if (pwr_m2 < 0) {
    analogWrite(motorPin1_m2, -pwr_m2);
    analogWrite(motorPin2_m2, 0);
  } else if (pwr_m2 == 0) {
    analogWrite(motorPin2_m2, 0);
    analogWrite(motorPin1_m2, 0);
  } else if (pwr_m2 > 0) {
    analogWrite(motorPin2_m2, pwr_m2);
    analogWrite(motorPin1_m2, 0);
  }


  if (pwr_m3 < 0) {
    analogWrite(motorPin1_m3, -pwr_m3);
    analogWrite(motorPin2_m3, 0);
  } else if (pwr_m3 == 0) {
    analogWrite(motorPin2_m3, 0);
    analogWrite(motorPin1_m3, 0);
  } else if (pwr_m3 > 0) {
    analogWrite(motorPin2_m3, pwr_m3);
    analogWrite(motorPin1_m3, 0);
  }


  if (pwr_m4 < 0) {
    analogWrite(motorPin1_m4, -pwr_m4);
    analogWrite(motorPin2_m4, 0);
  } else if (pwr_m4 == 0) {
    analogWrite(motorPin2_m4, 0);
    analogWrite(motorPin1_m4, 0);
  } else if (pwr_m4 > 0) {
    analogWrite(motorPin2_m4, pwr_m4);
    analogWrite(motorPin1_m4, 0);
  }


//  if (received_value_m1 > 0) {
//    analogWrite(motorPin1_m1, pwr_m1);
//    analogWrite(motorPin2_m1, 0);
//  } else if (received_value_m1 == 0) {
//    analogWrite(motorPin2_m1, 0);
//    analogWrite(motorPin1_m1, 0);
//  } else if (received_value_m1 < 0) {
//    analogWrite(motorPin2_m1, pwr_m1);
//    analogWrite(motorPin1_m1, 0);
//  }

//  if (received_value_m2 > 0) {
//    analogWrite(motorPin1_m2, pwr_m2);
//    analogWrite(motorPin2_m2, 0);
//  } else if (received_value_m2 == 0) {
//    analogWrite(motorPin2_m2, 0);
//    analogWrite(motorPin1_m2, 0);
//  } else if (received_value_m2 < 0) {
//    analogWrite(motorPin2_m2, pwr_m2);
//    analogWrite(motorPin1_m2, 0);
//  }

//  if (received_value_m3 > 0) {
//    analogWrite(motorPin1_m3, pwr_m3);
//    analogWrite(motorPin2_m3, 0);
//  } else if (received_value_m3 == 0) {
//    analogWrite(motorPin2_m3, 0);
//    analogWrite(motorPin1_m3, 0);
//  } else if (received_value_m3 < 0) {
//    analogWrite(motorPin2_m3, pwr_m3);
//    analogWrite(motorPin1_m3, 0);
//  }

//  if (received_value_m4 > 0) {
//    analogWrite(motorPin1_m4, pwr_m4);
//    analogWrite(motorPin2_m4, 0);
//  } else if (received_value_m4 == 0) {
//    analogWrite(motorPin2_m4, 0);
//    analogWrite(motorPin1_m4, 0);
//  } else if (received_value_m4 < 0) {
//    analogWrite(motorPin2_m4, pwr_m4);
//    analogWrite(motorPin1_m4, 0);
//  }

//  if (received_value_m1 == 200) {
//    wdt_enable(WDTO_15MS);
//    while (1) {}
//  }

  nh.spinOnce();
}
