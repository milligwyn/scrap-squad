// =============================================================================
// Capstone Project - Wall-ECE
// File:    Follower2_New.ino
// Author:  Dexter Fason
// Board:   Arduino Nano ESP32  (I2C address 2 — REAR wheels)
//
// Original architecture credit:
//   Curio Res on YT - https://youtube.com/@curiores111?si=Edl755O9VMZwGuMm
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// This board controls the TWO REAR WHEELS of the robot. It is structurally
// identical to Follower1_New.ino — the ONLY difference is the I2C address
// (#define I2C_ADDRESS 2 instead of 1), which the Leader uses to distinguish
// which follower it is addressing.
//
// If you need to make a logic change that applies to both followers (e.g. PID
// gain tuning, sync correction, I2C packet format), make it in BOTH files.
//
// -----------------------------------------------------------------------------
// THREADING MODEL
// -----------------------------------------------------------------------------
// The ESP32 has two cores. This program splits work between them:
//
//   Core 1 (Arduino loop thread)
//     - Runs setup() and loop().
//     - Handles I2C callbacks (requestEvent / receiveEvent). These are
//       triggered by hardware interrupts whenever the Leader talks to us.
//       Because the Wire library's interrupt handler runs on Core 1, the
//       I2C callbacks are effectively "free" — loop() itself stays empty.
//
//   Core 0 (FreeRTOS task: motorPIDLoop)
//     - Runs the PID controller for both motors at a fixed 100 Hz rate
//       (every 10 ms via vTaskDelay).
//     - Reads encoder positions, computes PID outputs, applies sync
//       correction, and drives the motor driver pins.
//
// Shared variables (pos[], target[], posi[]) are accessed by BOTH cores.
// The portMUX spinlock (mux) protects every read-modify-write of those
// variables so the two cores never corrupt each other's data.
//
// -----------------------------------------------------------------------------
// I2C COMMUNICATION PROTOCOL
// -----------------------------------------------------------------------------
// The Leader (master) communicates with this board (slave, address 2) using
// two types of I2C transaction:
//
//   WRITE (Leader → Follower2):
//     Leader sends 8 raw bytes = two 'long' values packed big-endian.
//       Bytes 0-3 → new target for motor 0 (rear-left)
//       Bytes 4-7 → new target for motor 1 (rear-right)
//     Handled by receiveEvent().
//
//   READ (Follower2 → Leader):
//     Leader requests 8 bytes. We respond with our current encoder positions.
//       Bytes 0-3 → pos[0] (rear-left encoder count)
//       Bytes 4-7 → pos[1] (rear-right encoder count)
//     Handled by requestEvent().
//
// -----------------------------------------------------------------------------
// PIN ASSIGNMENTS
// -----------------------------------------------------------------------------
//
//   ENCODER PINS:
//   enca[0]=17, encb[0]=18  — Encoder A/B for motor 0 (rear-left)
//   enca[1]=19, encb[1]=20  — Encoder A/B for motor 1 (rear-right)
//
//   D17–D20 map to GPIO1–GPIO4 (A0–A3 on the Nano ESP32). These pins were
//   chosen because:
//     - No special functions or peripheral conflicts.
//     - Not shared with the onboard RGB LED. D14=GPIO46 (Red), D15=GPIO0
//       (Green), D16=GPIO45 (Blue) were avoided. D15/GPIO0 is also an ESP32
//       boot strapping pin — if held LOW at power-on the chip enters download
//       mode instead of running the sketch.
//     - Not shared with I2C (SDA=D21/GPIO11, SCL=D22/GPIO12).
//     - All four support external interrupts.
//     - Previously this board used an RP2040 which has SDA/SCL on D18/D19.
//       Those pins are now used for encoder signals — do not reassign them
//       to I2C on this board.
//
//   ENCODER PULL-UPS:
//   NO external pull-up resistors are needed on ENCA/ENCB. The Hiwonder
//   Hall encoders use open-drain active-low outputs — they actively pull the
//   line LOW on each pulse and release it (float) when idle. INPUT_PULLUP
//   in setup() enables the ESP32's internal ~45kΩ pull-up which holds the
//   line HIGH when idle and is weak enough for the Hall sensor to pull LOW
//   cleanly on each trigger. External resistors of 1kΩ–4.7kΩ were found to
//   overpower the sensor and prevent it pulling to GND — do not add them.
//
//   INTERRUPT EDGE — FALLING:
//   The meaningful event on an active-low encoder is ENCA going HIGH→LOW
//   (FALLING) when the Hall sensor fires. Using FALLING ensures the ISR
//   triggers at the moment the magnet arrives, not when it leaves.
//
//   MOTOR DRIVER PINS:
//   pwm[0]=10 (D10/GPIO21) — ENA, right motor PWM speed
//   pwm[1]=5  (D5/GPIO8)   — ENB, left  motor PWM speed
//   in1[0]=9  (D9/GPIO18)  — IN1, right motor direction bit 1
//   in2[0]=8  (D8/GPIO17)  — IN2, right motor direction bit 2
//   in1[1]=7  (D7/GPIO10)  — IN3, left  motor direction bit 1
//   in2[1]=6  (D6/GPIO9)   — IN4, left  motor direction bit 2
//
//   I2C:
//   SDA=D21 (GPIO11), SCL=D22 (GPIO12) — Nano ESP32 default Wire pins.
//   Wire.begin(I2C_ADDRESS) uses these automatically.
// =============================================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Wire.h>

// I2C slave address for this board.
// Follower 1 = front wheels = address 1.
// Follower 2 = rear  wheels = address 2.
#define I2C_ADDRESS 2

// Hardware spinlock — protects shared variables between Core 0 and Core 1.
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


// =============================================================================
// SimplePID — Proportional-Integral-Derivative Controller
// =============================================================================
// Control law:  u = kp*e  +  kd*(de/dt)  +  ki*∫e dt
//
//   kp — Proportional gain. Higher = faster response but more overshoot.
//   kd — Derivative gain.   Higher = more damping, reduces overshoot.
//   ki — Integral gain.     Eliminates steady-state error; too high = windup.
//   umax — Maximum output (255 = full 8-bit PWM).
// =============================================================================
class SimplePID {
  private:
    float kp, kd, ki, umax;
    float eprev;      // Previous error — used to compute derivative term.
    float eintegral;  // Accumulated error × time — used for integral term.

  public:
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // evalu() — compute one PID step.
    //   value  — current encoder position
    //   target — desired encoder position
    //   deltaT — seconds since last call
    //   pwr    — OUTPUT: motor power magnitude (0–255)
    //   dir    — OUTPUT: direction (+1 forward, -1 reverse)
    void evalu(long value, long target, float deltaT, int &pwr, int &dir) {
      long  e    = target - value;
      float dedt = (e - eprev) / deltaT;
      eintegral  = eintegral + e * deltaT;
      float u    = kp * e + kd * dedt + ki * eintegral;

      pwr = (int)fabs(u);
      if (pwr > umax) pwr = umax;
      dir   = (u < 0) ? -1 : 1;
      eprev = (float)e;
    }
};


// =============================================================================
// Pin Definitions
// =============================================================================
#define NMOTORS 2

// Encoder pins — D17–D20 (GPIO1–GPIO4). No LED conflicts, no boot pin issues.
// ENCA is the interrupt channel; ENCB is sampled inside the ISR for direction.
const int enca[] = {17, 19};  // ENCA: motor 0 = D17 (GPIO1), motor 1 = D19 (GPIO3)
const int encb[] = {18, 20};  // ENCB: motor 0 = D18 (GPIO2), motor 1 = D20 (GPIO4)

// Motor driver pins.
const int pwm[]  = {10,  5};  // ENA=D10 (GPIO21, right), ENB=D5 (GPIO8, left)
const int in1[]  = { 9,  7};  // IN1=D9  (GPIO18, right), IN3=D7 (GPIO10, left)
const int in2[]  = { 8,  6};  // IN2=D8  (GPIO17, right), IN4=D6 (GPIO9,  left)


// =============================================================================
// Global Variables
// =============================================================================

// Last PID iteration timestamp (µs). Only accessed from motorPIDLoop — no lock.
long prevT = 0;

// posi[] — raw encoder counts incremented/decremented by the ISRs.
// 'long' avoids overflow on long paths (int overflows at ~50 m at this gear ratio).
// 'volatile' prevents the compiler from caching these in a register.
volatile long posi[] = {0, 0};

// pos[] — snapshot of posi[] taken at each PID tick, also sent to Leader on request.
volatile long pos[] = {0, 0};

// target[] — absolute encoder counts the motors should reach.
// Written by I2C receiveEvent (Core 1), read by PID task (Core 0).
volatile long target[] = {0, 0};

SimplePID    pid[NMOTORS];
TaskHandle_t MotorControlTask;


// =============================================================================
// setMotor() — H-bridge driver interface
// =============================================================================
//   dir=+1 → forward  (IN1 HIGH, IN2 LOW)
//   dir=-1 → reverse  (IN1 LOW,  IN2 HIGH)
//   dir= 0 → brake    (IN1 LOW,  IN2 LOW — motor coasts to stop)
// =============================================================================
void setMotor(int dir, int pwmVal, int pwmPin, int in1Pin, int in2Pin) {
  analogWrite(pwmPin, pwmVal);
  if (dir == 1) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (dir == -1) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}


// =============================================================================
// Encoder ISRs
// =============================================================================
// Triggered on FALLING edge of ENCA (active-low Hall encoder fires LOW).
// ENCB is sampled at the moment ENCA falls to determine direction:
//   ENCB HIGH when ENCA falls → motor spinning forward  → increment
//   ENCB LOW  when ENCA falls → motor spinning reverse  → decrement
//
// IRAM_ATTR: ISR executes from internal RAM, unaffected by flash cache misses.
// portENTER/EXIT_CRITICAL_ISR: prevents both ISRs from corrupting posi[]
// if they fire at the same time.
// =============================================================================
void IRAM_ATTR readEncoder0() {
  int b = digitalRead(encb[0]);
  portENTER_CRITICAL_ISR(&mux);
  posi[0] += (b > 0) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR readEncoder1() {
  int b = digitalRead(encb[1]);
  portENTER_CRITICAL_ISR(&mux);
  posi[1] += (b > 0) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&mux);
}


// =============================================================================
// I2C Serialisation Helpers
// =============================================================================
// Wire transfers one byte at a time. These helpers pack/unpack 4-byte 'long'
// values as big-endian streams so both Leader and Follower agree on byte order.
// =============================================================================
void sendLong(long value) {
  for (int k = 0; k < 4; k++) {
    Wire.write((value >> (8 * (3 - k))) & 0xFF);
  }
}

long receiveLong() {
  long outValue = 0;
  for (int k = 0; k < 4; k++) {
    outValue = (outValue << 8) | Wire.read();
  }
  return outValue;
}


// =============================================================================
// I2C Event Callbacks  (run on Core 1, triggered by Wire interrupt)
// =============================================================================
// Keep short — no delays, no Serial prints inside callbacks.

// requestEvent() — Leader is reading from us; send current encoder positions.
void requestEvent() {
  portENTER_CRITICAL(&mux);
  long tempPos0 = pos[0];
  long tempPos1 = pos[1];
  portEXIT_CRITICAL(&mux);
  sendLong(tempPos0);
  sendLong(tempPos1);
}

// receiveEvent() — Leader has written new targets to us; store them.
void receiveEvent(int howMany) {
  if (howMany < (int)(sizeof(long) * 2)) return; // Ignore short/corrupt packets.
  long tempTarget0 = receiveLong();
  long tempTarget1 = receiveLong();
  portENTER_CRITICAL(&mux);
  target[0] = tempTarget0;
  target[1] = tempTarget1;
  portEXIT_CRITICAL(&mux);
}


// =============================================================================
// setup()
// =============================================================================
void setup() {
  delay(100);

  // Start I2C as slave. Nano ESP32 defaults: SDA=D21 (GPIO11), SCL=D22 (GPIO12).
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  for (int k = 0; k < NMOTORS; k++) {
    // INPUT_PULLUP enables the ESP32's internal ~45kΩ pull-up.
    // This is the only pull-up needed for open-drain active-low Hall encoders.
    // Do NOT add external pull-up resistors — they overpower the Hall sensor
    // and prevent it from pulling the line LOW, causing missed pulses.
    pinMode(enca[k], INPUT_PULLUP);
    pinMode(encb[k], INPUT_PULLUP);

    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);

    // PID tuning — kp=1.5, kd=0.0, ki=0.05, max=255.
    // Adjust during physical testing if motors overshoot or oscillate.
    pid[k].setParams(1.5, 0.0, 0.05, 255);
  }

  // FALLING: triggers when ENCA goes HIGH→LOW (active-low encoder fires).
  // This is the correct edge for Hall encoders — the sensor pulls the line
  // LOW when a magnet passes, so FALLING = the moment of detection.
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder0, FALLING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder1, FALLING);

  // Pin PID task to Core 0. I2C callbacks and loop() run on Core 1.
  xTaskCreatePinnedToCore(motorPIDLoop, "MotorTask", 10000, NULL, 1, &MotorControlTask, 0);

  delay(1000);
}


// =============================================================================
// loop() — Core 1  (intentionally idle)
// =============================================================================
// Motor control runs in motorPIDLoop (Core 0).
// I2C is handled by interrupt-driven callbacks (Core 1).
// Small delay prevents the watchdog from triggering on an empty loop.
// =============================================================================
void loop() {
  delay(10);
}


// =============================================================================
// motorPIDLoop() — Core 0 FreeRTOS Task  (~100 Hz)
// =============================================================================
// 1. Compute deltaT for accurate PID math.
// 2. Snapshot shared encoder counts and targets under the spinlock.
// 3. Run independent PID for each motor.
// 4. Apply cross-motor sync correction to equalise wheel speeds.
// 5. Clamp PWM and drive the motor driver.
// =============================================================================
void motorPIDLoop(void* parameter) {
  for (;;) {

    // 1. Time delta
    long  currT  = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    prevT        = currT;

    // 2. Atomic snapshot
    portENTER_CRITICAL(&mux);
    long localPos0    = posi[0];
    long localPos1    = posi[1];
    pos[0]            = localPos0;
    pos[1]            = localPos1;
    long localTarget0 = target[0];
    long localTarget1 = target[1];
    portEXIT_CRITICAL(&mux);

    // 3. PID
    int pwr[NMOTORS];
    int dir[NMOTORS];
    pid[0].evalu(localPos0, localTarget0, deltaT, pwr[0], dir[0]);
    pid[1].evalu(localPos1, localTarget1, deltaT, pwr[1], dir[1]);

    // 4. Cross-motor sync
    // Clamped to ±50 to prevent a large startup gap from silencing a motor.
    // Skipped when motors run in opposite directions (e.g. turning).
    int   syncError = (int)constrain((long)(localPos0 - localPos1), -50L, 50L);
    float syncKp    = 0.8f;

    if (dir[0] == dir[1]) {
      if (dir[0] == 1) {
        pwr[0] -= (int)(syncError * syncKp);
        pwr[1] += (int)(syncError * syncKp);
      } else {
        pwr[0] += (int)(syncError * syncKp);
        pwr[1] -= (int)(syncError * syncKp);
      }
    }

    // 5. Apply
    for (int k = 0; k < NMOTORS; k++) {
      pwr[k] = constrain(pwr[k], 0, 255);
      setMotor(dir[k], pwr[k], pwm[k], in1[k], in2[k]);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
