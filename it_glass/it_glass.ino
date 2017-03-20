#include <SoftwareSerial.h> 

// blueTooth setting

int blueTx = 0;
int blueRx = 1;
SoftwareSerial mySerial(blueTx, blueRx);
String fromUser="";

// rgb led pin number
int redPin = 4;
int greenPin = 3;
int bluePin = 2;

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 
}
 
void loop()
{
  
  while(mySerial.available())
  {
    char myGetChar = (char)mySerial.read();
    fromUser+=myGetChar;
    delay(5);
  }
  /*
  if(mySerial.available()){
    Serial.write(mySerial.read());}
    if(Serial.available()){
      mySerial.write(Serial.read());}
  */
  if(!fromUser.equals("")){
    Serial.println("You said : "+fromUser);
  if(fromUser.equals("red"))
  {
    setColor(255, 0, 0);  // red
    delay(2000);
    }
  if(fromUser.equals("blue"))
  {
    
    setColor(0, 0, 255);  // blue
    delay(2000);
    }
    fromUser="";
  }
}
 
// RGB 값을 받아 analogWrite를 통해 각 핀에 연결된 LED에 전달 함수
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue); 
}
