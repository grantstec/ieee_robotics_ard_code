#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>

// TMC2209 Configuration
#define X_SERIAL Serial3  // Left motor
#define Z_SERIAL Serial1  // Right motor
#define R_SENSE 0.11f

TMC2209Stepper driverX(&X_SERIAL, R_SENSE, 0x00);  // Replace 0x00 with the actual driver address
TMC2209Stepper driverZ(&Z_SERIAL, R_SENSE, 0x01);  // Replace 0x01 with the actual driver address

// Stepper Pins
#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38

#define Z_STEP_PIN 46
#define Z_DIR_PIN 49
#define Z_ENABLE_PIN 62

// Stepper Objects
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// Constants
const unsigned long REPORT_INTERVAL = 100;
unsigned long last_report = 0;

void setup() {
    Serial.begin(115200);
    
    // TMC2209 Initialization
    X_SERIAL.begin(115200);
    Z_SERIAL.begin(115200);
    
    driverX.begin();
    driverX.pdn_disable(true);
    driverX.toff(5);
    driverX.en_spreadCycle(false);
    driverX.microsteps(16);
    driverX.irun(31);
    driverX.ihold(10);

    driverZ.begin();
    driverZ.pdn_disable(true);
    driverZ.toff(5);
    driverZ.en_spreadCycle(false);
    driverZ.microsteps(16);
    driverZ.irun(31);
    driverZ.ihold(10);

    // Stepper Configuration
    stepperX.setEnablePin(X_ENABLE_PIN);
    stepperX.setPinsInverted(true, false, true);
    stepperX.enableOutputs();
    stepperX.setMaxSpeed(12000);
    stepperX.setAcceleration(8000);

    stepperZ.setEnablePin(Z_ENABLE_PIN);
    stepperZ.setPinsInverted(true, false, true);
    stepperZ.enableOutputs();
    stepperZ.setMaxSpeed(12000);
    stepperZ.setAcceleration(8000);
}

void loop() {
    // Handle incoming commands
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("CMD:")) {
            cmd.remove(0, 4);
            int comma = cmd.indexOf(',');
            if (comma != -1) {
                long left = cmd.substring(0, comma).toInt();
                long right = cmd.substring(comma+1).toInt();
                
                stepperX.setSpeed(left);
                stepperZ.setSpeed(right);
            }
        }
    }

    // Run motors
    stepperX.runSpeed();
    stepperZ.runSpeed();

    // Report positions
    if (millis() - last_report > REPORT_INTERVAL) {
        Serial.print("STEPS:");
        Serial.print(stepperX.currentPosition());
        Serial.print(",");
        Serial.println(stepperZ.currentPosition());
        last_report = millis();
    }
}