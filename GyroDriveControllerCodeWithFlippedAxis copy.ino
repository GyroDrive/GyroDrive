#include <SPI.h> // Call SPI library for nRF24L01+ communication
#include <nRF24L01.h> // nRF2401 library
#include <RF24.h> // nRF2401 library
#include "I2Cdev.h" // MPU6050 Accelerometer library
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

int sensorPin = A0;   // Pin where the pressure sensor is connected
int pressValue;      // Variable to store the raw sensor value (0 - 1023)
const int pinCE = 9; // This pin sets the nRF24 to standby (0) or active mode (1)
const int pinCSN = 10; // Tells the nRF24 whether SPI communication is a command or message to send out
const int pinLED = 7; // Pin for LED

MPU6050 accelgyro; // Declare the object to access and control the accel
RF24 wirelessSPI(pinCE, pinCSN); // Create nRF24 object for wireless SPI connection
const uint64_t pAddress = 0xF00F1E5000LL; // Radio pipe address for 2-way communication

// Define packet structure
byte bArray[6]; // Byte array for data packet

void setup() {
    Serial.begin(9600); // Serial Monitor for tests
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    pinMode(2, INPUT_PULLUP); // Pin for Mode 1 (left)
    pinMode(4, INPUT_PULLUP); // Pin for Mode 2 (right)
    pinMode(pinLED, OUTPUT); // Pin for LED
    accelgyro.initialize(); // Initialize the accelerometer object
    wirelessSPI.begin(); // Start the nRF24 module
    wirelessSPI.setAutoAck(1); // Ensure autoACK is enabled
    wirelessSPI.enableAckPayload(); // Enable optional ack payloads
    wirelessSPI.setRetries(5, 15); // Set retries and timing for packets that were not ack'd
    wirelessSPI.openWritingPipe(pAddress); // Open writing pipe address
    wirelessSPI.stopListening(); // Set this nRF24 as transmitter
}

void loop() {
    // Read the switch state
    byte Mode;
    if (digitalRead(2) == LOW) {
        Mode = 1; // Left position, Mode 1
        digitalWrite(pinLED, HIGH); // Turn on LED
    } else if (digitalRead(4) == LOW) {
        Mode = 2; // Right position, Mode 2
        digitalWrite(pinLED, HIGH); // Turn on LED
    } else {
        Mode = 0; // Middle (off), no mode
        digitalWrite(pinLED, LOW); // Turn off LED
    }

    if (Mode != 0) { // If switch is not off
        int16_t tempX, tempY, z; // Temporary variables for raw accel values
        accelgyro.getAcceleration(&tempX, &tempY, &z); // Get accel values

        // Swap axes to match original orientation
        int16_t x = tempY; // Use y-axis value for x (forward/backward)
        int16_t y = tempX; // Use x-axis value for y (right/left)

        // Build packet data
        byte Press = buildPress();
        byte xValue = buildXValue(x);
        byte yValue = buildYValue(y);

        // Print values for Mode, Pressure, x and y
        Serial.print("Mode: ");
        Serial.print(Mode);
        Serial.print("  Pressure: ");
        Serial.print(Press);
        Serial.print("  X Value: ");
        Serial.print(xValue);
        Serial.print("  Y Value: ");
        Serial.println(yValue);

        // Build and send packet
        buildArray(Mode, Press, xValue, yValue);

        if (!wirelessSPI.write(bArray, 6)) { // If send fails, output to Serial Monitor
            Serial.println("Failed to send data");
        } else {
            Serial.println("Data sent");
        }
    }

    delay(5); // Delay before sending the next packet
}

// Build pressure sensor data
byte buildPress() {
    int pressValue = analogRead(sensorPin); // Read analog value from pressure sensor
    return map(pressValue, 0, 1023, 0, 250); // Map sensor value to range 0-250
}

// Build forward/backward direction and speed value
byte buildXValue(int xV) {
    if (xV <= 3000 && xV >= -3000) {
        return 125; // Stop value
    } else if (xV > 3000) {
        xV -= 3000;
        if (xV > 15000) xV = 15000; // Ceiling for forward speed
        return scaleSpeed(xV, 15000) + 125; // Scale and add 125 for forward
    } else {
        xV = abs(xV) - 3000;
        if (xV > 15000) xV = 15000; // Ceiling for backward speed
        return scaleSpeed(xV, 15000); // Scale for backward
    }
}

// Build right/left direction and angle value
byte buildYValue(int yV) {
    if (yV <= 3000 && yV >= -3000) {
        return 125; // Stop value
    } else if (yV > 3000) {
        yV -= 3000;
        if (yV > 11000) yV = 11000; // Ceiling for right angle
        return scaleSpeed(yV, 11000);
    } else {
        yV = abs(yV) - 3000;
        if (yV > 11000) yV = 11000; // Ceiling for left angle
        return scaleSpeed(yV, 11000) + 125; // Scale and add 125 for left
    }
}

// Scale accel speed value to fit in a byte
byte scaleSpeed(int scale, int sVal) {
    return (byte)((float)scale / sVal * 124); // Speed between 0 to 124
}

// Build packet with mode, pressure, x and y values
void buildArray(byte Mode, byte Press, byte xV, byte yV) {
    bArray[0] = 255; // Start of packet
    bArray[1] = Mode;
    bArray[2] = Press;
    bArray[3] = xV;
    bArray[4] = yV;
    bArray[5] = 254; // End of packet
}
