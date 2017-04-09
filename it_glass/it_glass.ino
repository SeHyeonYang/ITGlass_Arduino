#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SoftwareSerial.h>
//-------------------------------------HC-06 bluetooth setting.
int blueTx = 0; // tx=0
int blueRx = 1; // rx=1
SoftwareSerial btSerial(blueTx, blueRx);  // btSerial. use to bluetooth communication
String fromUser=""; // message fromg user
 
//------------------------------------RGB led pin number
int redPin = 4;
int greenPin = 3;
int bluePin = 5;
 
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars

Quaternion q;           // [w, x, y, z]         quaternion container
 
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float checkY[100];
float checkZ[100];
#define TEMP 20
int i=0;
int count =0 ;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
 
 
 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
 
 
 
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
 
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
   i=0;
  // bluetooth serial begin
  btSerial.begin(9600);
 
  // led pin setting
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
 
 
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
 
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 
    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
 
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
 
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
 
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
 
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}
 
 
 
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
 
void loop() {
 
    if (!dmpReady) return;
 
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
 
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

//     check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
     
 
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
 
        fifoCount -= packetSize;


 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            checkY[i] = ypr[1] * 180/M_PI;
            checkZ[i] = ypr[2] * 180/M_PI;
            //기울기 : 루트 [Y값제곱 x Z값제곱]
            int Gradient = sqrt( abs(checkY[i] - checkY[abs(i-TEMP)])*abs(checkY[i] - checkY[abs(i-TEMP)])+abs(checkZ[i] - checkZ[abs(i-TEMP)])*abs(checkZ[i] - checkZ[abs(i-TEMP)]));
            int absGradient = sqrt(ypr[1] * 180/M_PI*ypr[1] * 180/M_PI+ypr[2] * 180/M_PI*ypr[2] * 180/M_PI);
            
            if(absGradient>30 && count ==0)
           {   
              count = 1;
              setColor(0 , 0,255);
              Serial.print("drink\n");
              if(XMT.available()){
                btSerial.write(XMT.read());
                btSerial.flush();
              }
           }
           else if(absGradient<30 && count ==1)
           {
              
               count = 0;
               setColor(0, 0, 0);
                Serial.print("drank\n");
                if(XMT.available()){
                     btSerial.write(XMT.read());
                     btSerial.flush();
                }
           }
//           if(Gradient>28)
//           {      
//                
//                 Serial.print("Gradient ");
//                 if(Serial.available()){
//                      btSerial.write(Serial.read());
//                      btSerial.flush();
//                 }
//           }
//      
//            Serial.print("\t");
//            //Y값
//            Serial.print(ypr[1] * 180/M_PI);
//            
//            Serial.print("\t");
//            //Z값
//            Serial.println(ypr[2] * 180/M_PI);
             
            i++;
            if(i==100)
              i=0;
           
        #endif
 
 
 
 
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
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
