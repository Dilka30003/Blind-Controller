/**
 * Author Teemu Mäntykallio
 *
 * Plot TMC2130 or TMC2660 motor load using the stallGuard value.
 * You can finetune the reading by changing the STALL_VALUE.
 * This will let you control at which load the value will read 0
 * and the stall flag will be triggered. This will also set pin DIAG1 high.
 * A higher STALL_VALUE will make the reading less sensitive and
 * a lower STALL_VALUE will make it more sensitive.
 *
 * You can control the rotation speed with
 * 0 Stop
 * 1 Resume
 * + Speed up
 * - Slow down
 */
#include <TMCStepper.h>
#include <WiFi.h>

#include "Config.h"




// Select your stepper driver type
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);

using namespace TMC2208_n;

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

bool shaft = false;
bool homed = false;
int Position = 0;


void BackgroundThread() {
  
}

void homeBlind() {
  uint32_t ms = millis();
  uint32_t last_time=ms;
  driver.microsteps(32);
  driver.rms_current(RUNNING_CURRENT);
  shaft = false;
  driver.shaft(shaft);
  while (!homed) {
    //Serial.println("Homing");
    ms = millis();
    
    for (uint16_t i = 500; i > 0; i--) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(80);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(80);
        BackgroundThread();
    }
    int result = driver.SG_RESULT();
    Serial.println(result);
    if (result < STALL_VALUE && ms > last_time + 1000)
    {
      Serial.println("---------Homing Finished---------");
      homed = true;
    }
  }
  driver.microsteps(64);
  moveDown(HOME_OFFSET);
  driver.rms_current(IDLE_CURRENT);
  Position = 0;
}

void moveUp(int steps) {
  Serial.println("---------Opening---------");
  driver.rms_current(RUNNING_CURRENT);
  if (shaft != false) {
    shaft = false;
    driver.shaft(shaft);
    for (uint32_t i = BACKLASH; i > 0; i--) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(15);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(15);
    }
  }
  else {
    shaft = false;
    driver.shaft(shaft);
  }
  
  for (uint32_t i = steps; i > 0; i--) {
    Position -= 1;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(15);
  }
  driver.rms_current(IDLE_CURRENT);
  Serial.println("---------Opening Finished---------");
  Serial.println(Position);
}

void moveDown(int steps) {
  Serial.println("---------Closing---------");
  driver.rms_current(RUNNING_CURRENT);
  if (shaft != true) {
    shaft = true;
    driver.shaft(shaft);
    for (uint32_t i = BACKLASH; i > 0; i--) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(15);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(15);
    }
  }
  else {
    shaft = true;
    driver.shaft(shaft);
  }
  
  for (uint32_t i = steps; i > 0; i--) {
    Position += 1;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(15);
  }
  driver.rms_current(IDLE_CURRENT);
  Serial.println("---------Closing Finished---------");
  Serial.println(Position);
}


void setup() {
  Serial.begin(115200);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  SERIAL_PORT.begin(115200);
  //driver.beginSerial(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(IDLE_CURRENT);        // Set motor RMS current
  driver.microsteps(32);          // Set microsteps to 1/16th
//driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop

  driver.shaft(shaft);
  
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
  }

  homeBlind();
  delay(1000);
  moveDown(FULL_LENGTH);
  delay(1000);
  moveUp(FULL_LENGTH/2);
  delay(1000);
  moveDown(FULL_LENGTH/2);
  delay(1000);
  moveUp(FULL_LENGTH);
    
}

bool closeBlind = true;
bool finished = false;

void loop() {
  BackgroundThread();
}
