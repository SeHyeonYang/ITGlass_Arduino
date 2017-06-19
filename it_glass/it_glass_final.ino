#include "I2Cdev.h"
#include "HX711.h"
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
// Hx711.DOUT - pin #A2
// Hx711.SCK - pin #A3
 
#include <Hx711.h>
//Hx711 scale(A3, 6);
HX711 cell(A3, 6);
int horizon=0;
float pour=0;
float prePour=pour=0;
float maxA=0;
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
 
bool blinkState = false;
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
int absGradient;
int moveCheck;
float current=0;
 
// orientation/motion vars
 
Quaternion q;           // [w, x, y, z]         quaternion container
 
VectorFloat gravity;    // [x, y, z]            gravity vector
 
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float checkY;
float readyS;
float checkX;
float checkZ;
#define TEMP 20
int i=0;
int count =0 ;
int pourEnd=0;
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
//    pinMode(LED_PIN, OUTPUT);
}
 
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
 
void loop() {
 getMessageAndAct();
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
            checkX= ypr[0] * 180/M_PI;
            checkY = ypr[1] * 180/M_PI;
            checkZ = ypr[2] * 180/M_PI;
            //Í∏∞Ïö∏Í∏ : Î£®Ìä∏ [YÍ∞íÏ†úÍ≥ x ZÍ∞íÏ†úÍ≥ ]
            //int Gradient = sqrt( abs(checkY[i] - checkY[abs(i-TEMP)])*abs(checkY[i] - checkY[abs(i-TEMP)])+abs(checkZ[i] - checkZ[abs(i-TEMP)])*abs(checkZ[i] - checkZ[abs(i-TEMP)]));
            absGradient = sqrt(ypr[1]*180/M_PI * ypr[1] * 180/M_PI + ypr[2] * 180/M_PI*ypr[2] * 180/M_PI);
            moveCheck = sqrt(checkX*checkX + checkY * checkY + checkZ*checkZ);
 
 
 
            
            if(count == 0){
              if(maxA < 40){
                
                pourEnd = 0;
               while(pourEnd != 1){
                  getMessageAndAct();
                    pour=(cell.read()-240000)/2000;
                    if(pour < 500){
                      if(pour-prePour > 10 ){//æÁ¿Ã ¥√∞Ì¿÷¥Ÿ
                         //Serial.println("pouring");
                         
                         delay(1000);
                         pourEnd = 2;
                       }
                        else if(pour-prePour < 10 && pourEnd==2){// ¥Ÿµ˚∂˙¥Ÿ
                         maxA=pour;
                          pourEnd = 1;
                       }
                     prePour=pour;
                   }
               }
                 //Serial.print("maxA is ");
                 //Serial.println(maxA);
              }
            }
              //  Serial.println(pour);
 
            
            if(absGradient>30 && count ==0)
           {   
              count = 1;
              getMessageAndAct();
              Serial.print("drink\n");
              if(Serial.available()){
                btSerial.write(Serial.read());
                btSerial.flush();
              }
              horizon=0;
           }
           else if(absGradient<30 && count ==1)
           {
               count = 0;
                Serial.print("drank\n");
                if(Serial.available()){
                     btSerial.write(Serial.read());
                     btSerial.flush();
                }
                delay(3000);
                current=(cell.read()-240000)/2000;
             
                  if(current < maxA){
                   Serial.println(maxA-current);
                  if(Serial.available()){
                      btSerial.write(Serial.read());
                      btSerial.flush();
                  }
                  maxA = current;
                  }
                
                  
                
           }
//            Serial.println(checkX);
//            
//           if(checkX-0.5<readyS&&readyS<checkX+0.5){
//              pour=(cell.read()-245000)/2000;
//              if(pour > current+20){
//                delay(300);
//                Serial.println(scale.getGram(), 1);
//              }
//                if(Serial.available()){
//                     btSerial.write(Serial.read());
//                     btSerial.flush();
//                }
//           }
//     //     else  if(moveCheck>20)
//     //      horizon=0;
////            readyS = checkX ;
 
             
        
           
        #endif
 
 
 
 
        // blink LED to indicate activity
//        blinkState = !blinkState;
//        digitalWrite(LED_PIN, blinkState);
    }
}
// RGB Í∞íÏùÑ Î∞õÏïÑ analogWriteÎ• Üµ ï¥ Í∞ óê ó∞Í≤∞Îêú LED óê †Ñ ã¨
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue); 
}
 
void getMessageAndAct(){
 
  // ì§ ñ¥ ò® Î©îÏãúÏß Í∞ ûà úºÎ© Î∞õÎäî ã§.
  while(btSerial.available())
  {
    char myGetChar = (char)btSerial.read();
    fromUser+=myGetChar;
    delay(5);
  } 
 
  // Î©îÏãúÏß óê î∞Î• ï° Öò 
  if(!fromUser.equals("")){
 
    // ï± úºÎ°úÎ Ñ∞ ä§ ä∏Îß r.g.b Î• Í∞ÅÍ∞Å int àòÏπòÎ°ú ñª äî ã§. 
    char * color = fromUser.c_str();
    char * first = strtok(color, ".");
    char * second = strtok(NULL, ".");
    char * third = strtok(NULL, ".");
 
    int r = atoi(first);
    int g = atoi(second);
    int b = atoi(third);
 
    setColor(r,g,b);
 
    delay(2000);
 
      setColor(0,0,0);
 
    fromUser="";
  }
  }
