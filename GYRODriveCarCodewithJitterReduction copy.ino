#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

const int escPin = 5;  // ESC signal pin
const int servoPin = 3;  // Servo signal pin
Servo myServo;
Servo esc;  // Create Servo object for ESC

const int pinCE = 9;
const int pinCSN = 10;
byte bArray[6];
RF24 wirelessSPI(pinCE, pinCSN);
const uint64_t pAddress = 0xF00F1E5000LL;
int sCount = 0;

int prevServoAngle = 90; // Initial value for servo
int jitterThreshold = 5; // Jitter sensitivity for the servo

void setup() {
  Serial.begin(9600);
  
  wirelessSPI.begin();
  wirelessSPI.setAutoAck(1);
  wirelessSPI.enableAckPayload();
  wirelessSPI.setRetries(5, 10);
  wirelessSPI.openReadingPipe(1, pAddress);
  wirelessSPI.startListening();
  motorStop();  // Stop motor at the start

  pinMode(escPin, OUTPUT);
  myServo.attach(servoPin);
  esc.attach(escPin);  // Attach ESC to pin

  // Initialize ESC
  Serial.println("Initializing ESC...");
  esc.write(0);  // Send low signal to arm ESC
  delay(2000);  // Wait for ESC to arm
  Serial.println("ESC Ready");
}

void loop() {
  if (wirelessSPI.available()) {
    wirelessSPI.read(bArray, 6);
    Serial.print("Message Received: ");
    
    for (int i = 0; i < 6; i++) {
      Serial.print(bArray[i]);
      if (i < 5) {
        Serial.print(", ");
      }
    }

    // Move to the next line after printing the array
    Serial.println();
    if (verifyPacket()) {
      if (bArray[1] == 1) {
        Mode1(bArray[3], bArray[4]);
      } else if (bArray[1] == 2) {
        Mode2(bArray[2], bArray[3], bArray[4]);
      } else {
        motorStop();
      }
      sCount = 0;
    }
  }

  delay(10);
  sCount++;
  if (sCount > 60) {
    motorStop();
  }
}

bool verifyPacket() {
  return (bArray[0] == 255 && bArray[5] == 254 && bArray[1] < 251 && bArray[2] < 251 && bArray[3] < 251 && bArray[4] < 251);
}

void Mode1(byte xAxis, byte yAxis) {
  int motorSpeed;
  int servoAngle;

  // Motor control (no jitter threshold applied)
  if (xAxis < 125) {
    motorSpeed = map(xAxis, 0, 124, 88, 60);  // Range of 70% Max reverse throttle to min
    esc.write(motorSpeed); // Reverse Direction
  } else if (xAxis == 125) {
    esc.write(94);  // Stop motor
  } else if (xAxis >= 126) {
    motorSpeed = map(xAxis, 126, 250, 98, 108);
    esc.write(motorSpeed);  // Forward direction
  }

  // Servo control with jitter reduction
  if (yAxis < 125) {
    servoAngle = map(yAxis, 0, 124, 89, 70);
  } else if (yAxis == 125) {
    servoAngle = 90;
  } else if (yAxis >= 126) {
    servoAngle = map(yAxis, 126, 250, 91, 110);
  }

  // Apply jitter threshold to servo angle
  if (abs(servoAngle - prevServoAngle) > jitterThreshold) {
    myServo.write(servoAngle);
    prevServoAngle = servoAngle;  // Update previous angle after significant change
  }
}

void Mode2(byte pressure, byte xAxis, byte yAxis) {
  int motorSpeed;
  int servoAngle;

  // Motor control (no jitter threshold applied)
  if (xAxis < 125) { // Indicates reverse direction
    motorSpeed = map(pressure, 50, 255, 88, 60);
  } else if (xAxis >= 126) { // Indicates forward direction
    motorSpeed = map(pressure, 50, 255, 98, 105);
  } else { // Indicates threshold area
    motorSpeed = 94;
  }

  esc.write(motorSpeed);

  // Servo control with jitter reduction
  if (yAxis < 125) {
    servoAngle = map(yAxis, 0, 124, 89, 70);
  } else if (yAxis == 125) {
    servoAngle = 90;
  } else if (yAxis >= 126) {
    servoAngle = map(yAxis, 126, 250, 91, 110);
  }

  // Apply jitter threshold to servo angle
  if (abs(servoAngle - prevServoAngle) > jitterThreshold) {
    myServo.write(servoAngle);
    prevServoAngle = servoAngle;  // Update previous angle after significant change
  }
}

void motorStop() {
  esc.write(94);  // Stop motor
}
