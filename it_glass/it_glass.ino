#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE    //defined in "I2Cdev.h"
#include "Wire.h"
#endif
 
//-------------------------------------HC-06 bluetooth setting.
int blueTx = 0; // tx=0
int blueRx = 1; // rx=1
SoftwareSerial btSerial(blueTx, blueRx);  // btSerial. use to bluetooth communication
String fromUser=""; // message fromg user

//------------------------------------RGB led pin number
int redPin = 4;
int greenPin = 3;
int bluePin = 5;
 
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

  // bluetooth serial begin
  btSerial.begin(9600);

  // led pin setting
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
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
  getMessageAndAct();
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
    keepSendCnt = KEEP_SEND_TIMES;
    MPU6050ThresholdCap = 0;
  }
  else{
    if(noMotionCnt) noMotionCnt--;
    if(!noMotionCnt && keepSendCnt){
      keepSendCnt--;
      noMotionCnt = MOTION_DETECTION_HZ;  //motion
    }  
  }
}  
 
 
void sendMotion(void){
    XMT.println("컵이 움직인다.");
      if(XMT.available()){
      btSerial.write(XMT.read());
      btSerial.flush();
    }
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

// RGB 값을 받아 analogWrite를 통해 각 핀에 연결된 LED에 전달
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue); 
}

void getMessageAndAct(){

  // 들어온 메시지가 있으면 받는다.
  while(btSerial.available())
  {
    char myGetChar = (char)btSerial.read();
    fromUser+=myGetChar;
    delay(5);
  } 

  // 메시지에 따른 액션 
  if(!fromUser.equals("")){

    // 앱으로부터 스트링 r.g.b 를 각각 int 수치로 얻는다. 
    char * color = fromUser.c_str();
    char * first = strtok(color, ".");
    char * second = strtok(NULL, ".");
    char * third = strtok(NULL, ".");

    int r = atoi(first);
    int g = atoi(second);
    int b = atoi(third);

    setColor(r,g,b);

    delay(2000);
 
    fromUser="";
  }
  }

  /*
   Serial.println(fromUser);
    if(Serial.available()){
      btSerial.write(Serial.read());
      btSerial.flush();
    }
  */
