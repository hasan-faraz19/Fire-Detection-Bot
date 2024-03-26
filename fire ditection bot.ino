#include <Servo.h>             // Include the Servo library
#include <SoftwareSerial.h>    // Include the SoftwareSerial library

Servo fireServo;

int servoPos = 0;    
boolean fireDetected = false;

const String phoneNumber = "01*********"; // Use your number with country code

#define rxPin 2
#define txPin 3 
SoftwareSerial gsmModule(rxPin, txPin);
#define leftSensorPin 4          // Left sensor pin
#define rightSensorPin 5         // Right sensor pin
#define forwardSensorPin 6       // Front sensor pin
#define gasSensorPin 7           // Gas sensor pin
#define leftMotor1Pin 8          // Left motor pin 1
#define leftMotor2Pin 9          // Left motor pin 2
#define rightMotor1Pin 10        // Right motor pin 1
#define rightMotor2Pin 11        // Right motor pin 2
#define pumpPin 12               // Water pump pin

void setup() {
  Serial.begin(115200);
  gsmModule.begin(9600);
  gsmModule.println("AT");
  delay(1000);
  gsmModule.println("AT+CMGF=1");
  delay(1000);
  
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  pinMode(forwardSensorPin, INPUT);
  pinMode(gasSensorPin, INPUT);
  pinMode(leftMotor1Pin, OUTPUT);
  pinMode(leftMotor2Pin, OUTPUT);
  pinMode(rightMotor1Pin, OUTPUT);
  pinMode(rightMotor2Pin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
 
  fireServo.attach(13);
  fireServo.write(90); 
  
  while(gsmModule.available()) {
    Serial.println(gsmModule.readString());
  }
}

void extinguishFire() {
  digitalWrite(leftMotor1Pin, HIGH);
  digitalWrite(leftMotor2Pin, HIGH);
  digitalWrite(rightMotor1Pin, HIGH);
  digitalWrite(rightMotor2Pin, HIGH);
  digitalWrite(pumpPin, HIGH);
  delay(500);
 
  for (servoPos = 50; servoPos <= 110; servoPos += 1) { 
    fireServo.write(servoPos); 
    delay(10);  
  }
  for (servoPos = 110; servoPos >= 50; servoPos -= 1) { 
    fireServo.write(servoPos); 
    delay(10);
  }
  digitalWrite(pumpPin, LOW);
  fireServo.write(90); 
  fireDetected = false;
}

void loop() {
  fireServo.write(90);
  
  if (digitalRead(leftSensorPin) == 1 && digitalRead(rightSensorPin) == 1 && digitalRead(forwardSensorPin) == 1) {
    delay(500);
    digitalWrite(leftMotor1Pin, HIGH);
    digitalWrite(leftMotor2Pin, HIGH);
    digitalWrite(rightMotor1Pin, HIGH);
    digitalWrite(rightMotor2Pin, HIGH);
  }
  else if (digitalRead(forwardSensorPin) == 0) {
    digitalWrite(leftMotor1Pin, HIGH);
    digitalWrite(leftMotor2Pin, LOW);
    digitalWrite(rightMotor1Pin, HIGH);
    digitalWrite(rightMotor2Pin, LOW);
    fireDetected = true;
  }
  else if (digitalRead(leftSensorPin) == 0) {
    digitalWrite(leftMotor1Pin, HIGH);
    digitalWrite(leftMotor2Pin, LOW);
    digitalWrite(rightMotor1Pin, HIGH);
    digitalWrite(rightMotor2Pin, HIGH);
  }
  else if (digitalRead(rightSensorPin) == 0) {
    digitalWrite(leftMotor1Pin, HIGH);
    digitalWrite(leftMotor2Pin, HIGH);
    digitalWrite(rightMotor1Pin, HIGH);
    digitalWrite(rightMotor2Pin, LOW);
  }
  delay(400); // Change this value to change the distance
  
  if (digitalRead(gasSensorPin) == 0) {
    Serial.println("Gas is Detected.");
    sendSMS(); 
  }
   
  while (fireDetected == true) {
    extinguishFire();
    Serial.println("Fire Detected.");
    makeCall();
  }
}

void makeCall() {
  Serial.println("Calling....");
  gsmModule.println("ATD" + phoneNumber + ";");
  delay(20000); // 20 sec delay
  gsmModule.println("ATH");
  delay(1000); // 1 sec delay
}

void sendSMS() {
  Serial.println("Sending SMS....");
  delay(50);
  gsmModule.print("AT+CMGF=1\r");
  delay(1000);
  gsmModule.print("AT+CMGS=\"" + phoneNumber + "\"\r");
  delay(1000);
  gsmModule.print("Gas Detected");
  delay(100);
  gsmModule.write(0x1A);
  delay(5000);
}
