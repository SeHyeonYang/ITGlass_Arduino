#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE    //defined in "I2Cdev.h"
#include "Wire.h"
#endif
 
//-------------------------------------HC-06
SoftwareSerial SoftSerial(2, 3);  //TX=2, RX=3
 
#define XMT   Serial
//#define XMT   softSerial
 
//-------------------------------------MPU6050
MPU6050 accelgyro;
 
#define MPU6050_THRESHOLD (500.0) //(1000.0)
#define MPU6050_DETECTION_SPEED  (1000/30)   //30hz
 
long MPU6050Timer;
byte MPU6050ThresholdCap;
 
int16_t ax, ay, az;
int16_t gx, gy, gz;
 
int16_t pre_ax, pre_ay, pre_az;
int16_t event_ax, event_ay, event_az;
int16_t event_gx, event_gy, event_gz;
 
int16_t pipe[4][6];   //4tab filter
byte pipeIx;
 
//-------------------------------------Detection
long detectionTimer;
byte noMotionCnt;
byte keepSendCnt;
 
#define MOTION_DETECTION_HZ    (10) 
#define MOTION_DETECTION_SPEED  (1000/MOTION_DETECTION_HZ)
#define KEEP_SEND_TIMES         (10)    //event  10 .
 
  /*---------------------------------------------------------
      S  E  T  U  P
  ---------------------------------------------------------*/
void setup() {
  Serial.begin(9600);
  SoftSerial.begin(9600);
 
  MPU6050Timer = 0; //long
  detectionTimer = 0; //long
  MPU6050ThresholdCap = 0; //byte
  noMotionCnt = MOTION_DETECTION_HZ; //byte = 10
  pipeIx = 0;
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.println("Initializing I2C devices...");
 
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}
 
  /*---------------------------------------------------------
      L  O  O  P
  ---------------------------------------------------------*/
void loop() {
  getAccel();
  getMotion();
}
 
 
//-----------------------------------------------
void getMotion(void){
 
  long t = millis();
  if((t- detectionTimer) < MOTION_DETECTION_SPEED) return;
  detectionTimer = t;
 
  if(MPU6050ThresholdCap){    // 6050 event ߻ .
    sendMotion();
    XMT.println("컵이 움직인다.");
    keepSendCnt = KEEP_SEND_TIMES;
    MPU6050ThresholdCap = 0;
  }
  else{
    if(noMotionCnt) noMotionCnt--;
    if(!noMotionCnt && keepSendCnt){
      sendMotion();
      XMT.println("컵이 안 움직인다.");
      keepSendCnt--;
      noMotionCnt = MOTION_DETECTION_HZ;  //motion  1 ʿ 1ȸ Ѵ .
    }  
  }
}  
 
 
void sendMotion(void){
  XMT.print(event_gx);
  XMT.write(',');
  XMT.print(event_gy);
  XMT.write(',');
  XMT.print(event_gz);
  XMT.println(",");
}
 
//-----------------------------------------------
void getAccel(void){
  float SVM;
  float x,y,z;
 
  long t = millis();
  if((t- MPU6050Timer) < MPU6050_DETECTION_SPEED) return;
  MPU6050Timer = t;
 
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  pipe[pipeIx][0] = ax;
  pipe[pipeIx][1] = ay;
  pipe[pipeIx][2] = az;
  pipe[pipeIx][3] = gx;
  pipe[pipeIx][4] = gy;
  pipe[pipeIx][5] = gz;
  pipeIx++;
  pipeIx &= 0x03; //4tab filter
 
  ax = tab4(0);
  ay = tab4(1);
  az = tab4(2);  
  gx = tab4(3);
  gy = tab4(4);
  gz = tab4(5);  
 
  x = pre_ax - ax;
  y = pre_ay - ay;
  z = pre_az - az;
 
  SVM = sqrt( x*x + y*y + z*z );  
  pre_ax = ax;
  pre_ay = ay;
  pre_az = az;
 
  if(SVM >= MPU6050_THRESHOLD){
    event_ax = ax;
    event_ay = ay;
    event_az = az;
 
    accelgyro.getRotation(&gx, &gy, &gz);
    event_gx = gx;
    event_gy = gy;
    event_gz = gz;
 
    MPU6050ThresholdCap++;
  }
}  
 
//--------------------
int tab4(byte i){
  long t;
  t = pipe[0][i];
  t += pipe[1][i];
  t += pipe[2][i];
  t += pipe[3][i];  
  t /= 4;
  return (int)t;
}
