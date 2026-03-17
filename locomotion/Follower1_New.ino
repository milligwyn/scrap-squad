// =============================================================================
// Capstone Project - Wall-ECE
// File:    Follower1_New.ino
// Author:  Dexter Fason
// Board:   Arduino Nano ESP32  (I2C address 1 — FRONT wheels)
//
// Original architecture credit:
//   Curio Res on YT - https://youtube.com/@curiores111?si=Edl755O9VMZwGuMm
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// This board controls the TWO FRONT WHEELS of the robot. It receives encoder
// targets from the Leader (Giga R1 M4 core) over I2C and drives both motors to
// those targets using independent PID controllers. A secondary sync correction
// keeps the two front wheels running at the same speed even if one drifts.
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
// The Leader (master) communicates with this board (slave, address 1) using
// two types of I2C transaction:
//
//   WRITE (Leader → Follower1):
//     Leader sends 8 raw bytes = two 'long' values packed big-endian.
//       Bytes 0-3 → new target for motor 0 (front-left)
//       Bytes 4-7 → new target for motor 1 (front-right)
//     Handled by receiveEvent().
//
//   READ (Follower1 → Leader):
//     Leader requests 8 bytes. We respond with our current encoder positions.
//       Bytes 0-3 → pos[0] (front-left encoder count)
//       Bytes 4-7 → pos[1] (front-right encoder count)
//     Handled by requestEvent().
//
// -----------------------------------------------------------------------------
// PIN ASSIGNMENTS
// -----------------------------------------------------------------------------
//   enca[0]=14, encb[0]=15  — Encoder A/B channels for motor 0 (front-left)
//   enca[1]=16, encb[1]=17  — Encoder A/B channels for motor 1 (front-right)
//   pwm[0]=5,   pwm[1]=10   — PWM speed pins for motor driver
//   in1[0]=9,   in1[1]=7    — Direction pin 1 for each motor
//   in2[0]=8,   in2[1]=6    — Direction pin 2 for each motor
// =============================================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Wire.h>

// I2C slave address for this board. The Leader uses this address to address
// the front-wheel follower specifically. Follower 2 uses address 2.
#define I2C_ADDRESS 1

// Hardware spinlock used to safely share variables between Core 0 and Core 1.
// portENTER_CRITICAL / portEXIT_CRITICAL disable interrupts on the calling core
// and spin-wait if the other core holds the lock, guaranteeing atomic access.
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


// =============================================================================
// SimplePID — Proportional-Integral-Derivative Controller
// =============================================================================
// Computes a motor power and direction given a current position, a target
// position, and the elapsed time since the last computation (deltaT).
//
// Control law:  u = kp*e  +  kd*(de/dt)  +  ki*∫e dt
//
//   kp — Proportional gain. Higher = faster response but more overshoot.
//   kd — Derivative gain.   Higher = more damping, reduces overshoot.
//   ki — Integral gain.     Higher = eliminates steady-state error faster,
//                           but too high causes integral windup oscillation.
//   umax — Maximum output value (255 for 8-bit PWM).
// =============================================================================
class SimplePID {
  private:
    float kp, kd, ki, umax;
    float eprev;      // Error from the previous iteration (for derivative term).
    float eintegral;  // Running sum of error × time (for integral term).

  public:
    // Default constructor sets safe gains — actual values set via setParams().
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

    // setParams() — call once during setup() to configure the controller.
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // evalu() — compute one PID step.
    //
    //   value  — current encoder position (long for overflow safety)
    //   target — desired encoder position
    //   deltaT — seconds since last call (used to scale derivative/integral)
    //   pwr    — OUTPUT: motor power magnitude (0–255)
    //   dir    — OUTPUT: motor direction (+1 = forward, -1 = reverse)
    void evalu(long value, long target, float deltaT, int &pwr, int &dir) {
      long  e    = target - value;           // Positional error.
      float dedt = (e - eprev) / deltaT;    // Rate of error change (derivative).
      eintegral  = eintegral + e * deltaT;  // Accumulated error over time (integral).

      float u = kp * e + kd * dedt + ki * eintegral;  // PID output.

      pwr = (int)fabs(u);           // Magnitude → PWM duty cycle.
      if (pwr > umax) pwr = umax;   // Clamp to hardware maximum.

      dir = (u < 0) ? -1 : 1;      // Sign → motor direction.

      eprev = (float)e;  // Store error for next derivative calculation.
    }
};


// =============================================================================
// Pin Definitions and Global Variables
// =============================================================================

#define NMOTORS 2  // This board controls exactly 2 motors.

// Encoder pins — enca is the interrupt-driven channel; encb gives direction.
const int enca[] = {14, 16};
const int encb[] = {15, 17};

// Motor driver pins.
const int pwm[]  = {5,  10};  // PWM speed (analogWrite).
const int in1[]  = {9,   7};  // Direction bit 1 (digitalWrite).
const int in2[]  = {8,   6};  // Direction bit 2 (digitalWrite).

// Timestamp of the last PID iteration (microseconds). Used to compute deltaT.
// Only ever accessed from motorPIDLoop (Core 0), so no locking needed.
long prevT = 0;

// posi[] — raw encoder counts accumulated by the ISRs.
// Declared volatile because they are written by interrupt handlers and read
// by the PID task on a different core. 'long' prevents overflow on long paths.
volatile long posi[] = {0, 0};

// pos[] — snapshot of posi[] taken at the start of each PID iteration.
// Shared between the PID task (writer) and the I2C requestEvent (reader).
// Access must be protected by the spinlock.
volatile long pos[] = {0, 0};

// target[] — the absolute encoder counts each motor should reach.
// Written by the I2C receiveEvent (Core 1) and read by the PID task (Core 0).
// Access must be protected by the spinlock.
volatile long target[] = {0, 0};

// One PID controller instance per motor.
SimplePID pid[NMOTORS];

// Handle for the FreeRTOS motor task (used to delete/suspend it if needed).
TaskHandle_t MotorControlTask;


// =============================================================================
// setMotor() — Motor Driver Interface
// =============================================================================
// Writes to a standard H-bridge motor driver (e.g. L298N, TB6612).
//
//   dir    : +1 = forward, -1 = reverse, 0 = brake (both IN pins LOW).
//   pwmVal : speed 0–255.
//   pwmPin, in1Pin, in2Pin : hardware pin numbers for this motor.
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
    // dir == 0: both LOW = brake (motor coasts to a stop).
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}


// =============================================================================
// Encoder ISRs — Interrupt Service Routines
// =============================================================================
// Triggered on the RISING edge of each encoder's A channel (enca).
// Reads the B channel (encb) to determine direction:
//   encb HIGH when A rises → forward  → increment
//   encb LOW  when A rises → reverse  → decrement
//
// IRAM_ATTR places the ISR in internal RAM so it runs even if the flash cache
// is busy — essential for reliable interrupt response on the ESP32.
//
// portENTER_CRITICAL_ISR / portEXIT_CRITICAL_ISR protect the increment from
// being interrupted by the other encoder's ISR mid-update.
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
// The Wire library transfers data one byte at a time, but encoder positions are
// 'long' (4 bytes). These helpers pack/unpack longs as big-endian byte streams.
//
// Big-endian means the most-significant byte is sent first.
// Both the Leader and Followers use the same byte order, so they agree.
// =============================================================================

// sendLong() — serialize a long into 4 bytes and queue them for I2C send.
// Called only from within a requestEvent callback (Wire is already active).
void sendLong(long value) {
  for (int k = 0; k < 4; k++) {
    // Shift the desired byte into the lowest 8 bits, then mask to 1 byte.
    Wire.write((value >> (8 * (3 - k))) & 0xFF);
  }
}

// receiveLong() — read 4 bytes from the I2C receive buffer and reassemble them
// into a long. Called only from within a receiveEvent callback.
long receiveLong() {
  long outValue = 0;
  for (int k = 0; k < 4; k++) {
    outValue = (outValue << 8) | Wire.read(); // Shift existing bits up, OR in next byte.
  }
  return outValue;
}


// =============================================================================
// I2C Event Callbacks
// =============================================================================
// These functions are registered with Wire and called automatically by the
// Wire interrupt handler (on Core 1) whenever the Leader addresses this board.
// Keep them short — no delays, no Serial prints inside callbacks.
// =============================================================================

// requestEvent() — called when the Leader asks US to send data (a READ transaction).
// We respond with our two current encoder positions so the Leader can check
// whether we have reached our targets.
void requestEvent() {
  // Take a spinlock snapshot of pos[] so the PID task cannot update it
  // mid-transmission (a half-updated long would produce garbage readings).
  portENTER_CRITICAL(&mux);
  long tempPos0 = pos[0];
  long tempPos1 = pos[1];
  portEXIT_CRITICAL(&mux);

  sendLong(tempPos0);
  sendLong(tempPos1);
}

// receiveEvent() — called when the Leader WRITES new target positions to us.
// howMany is the byte count received; we guard against truncated packets.
void receiveEvent(int howMany) {
  if (howMany < (int)(sizeof(long) * 2)) return; // Ignore incomplete packets.

  long tempTarget0 = receiveLong();
  long tempTarget1 = receiveLong();

  // Store new targets under the spinlock so the PID task sees them atomically.
  portENTER_CRITICAL(&mux);
  target[0] = tempTarget0;
  target[1] = tempTarget1;
  portEXIT_CRITICAL(&mux);
}


// =============================================================================
// setup()
// =============================================================================
void setup() {
  delay(100); // Brief stabilisation delay after power-on.

  // Start I2C as a SLAVE with our assigned address.
  // The callbacks registered below are invoked automatically by the Wire ISR.
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);  // Leader wants to read our positions.
  Wire.onReceive(receiveEvent);  // Leader is sending us new targets.

  // Configure all motor and encoder pins, and set initial PID gains.
  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT_PULLUP); // Encoder A channel (interrupt source).
    pinMode(encb[k], INPUT_PULLUP); // Encoder B channel (direction sense).
    pinMode(pwm[k],  OUTPUT);
    pinMode(in1[k],  OUTPUT);
    pinMode(in2[k],  OUTPUT);

    // PID tuning values — kp=1.5, kd=0.0, ki=0.05, max_output=255.
    // Adjust these during physical testing if motors overshoot or oscillate.
    pid[k].setParams(1.5, 0.0, 0.05, 255);
  }

  // Attach ISRs to the encoder A channels. RISING = trigger when A goes HIGH.
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder1, RISING);

  // Create the PID task and pin it to Core 0.
  // Core 1 is reserved for the Arduino loop() and I2C interrupt handling.
  // Stack size 10000 bytes, priority 1 (low — yield to interrupts freely).
  xTaskCreatePinnedToCore(motorPIDLoop, "MotorTask", 10000, NULL, 1, &MotorControlTask, 0);

  delay(1000); // Allow PID task to start before the system begins receiving commands.
}


// =============================================================================
// loop() — Core 1
// =============================================================================
// Intentionally empty. All real work happens in:
//   - motorPIDLoop task (Core 0) for motor control.
//   - requestEvent / receiveEvent callbacks (Core 1, interrupt-driven) for I2C.
// The small delay prevents the watchdog timer from triggering on an empty loop.
// =============================================================================
void loop() {
  delay(10);
}


// =============================================================================
// motorPIDLoop() — Core 0 FreeRTOS Task
// =============================================================================
// Runs at approximately 100 Hz (every 10 ms). Each iteration:
//   1. Computes deltaT (time since last iteration) for accurate PID math.
//   2. Snapshots the shared encoder counts and targets under the spinlock.
//   3. Runs the PID controller for each motor independently.
//   4. Applies a cross-motor sync correction to keep both wheels in step.
//   5. Clamps PWM values and drives the motor driver pins.
// =============================================================================
void motorPIDLoop(void* parameter) {
  for (;;) {

    // --- 1. Compute time delta -------------------------------------------
    long  currT  = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6; // Convert µs → seconds.
    prevT        = currT;

    // --- 2. Atomic snapshot of shared variables --------------------------
    // We copy posi[] (written by ISRs) into pos[] (used by I2C callbacks)
    // and grab local copies of targets — all inside one critical section to
    // guarantee a consistent snapshot that can't be torn by an interrupt.
    portENTER_CRITICAL(&mux);
    long localPos0    = posi[0];
    long localPos1    = posi[1];
    pos[0]            = localPos0;  // Update pos[] for the I2C requestEvent.
    pos[1]            = localPos1;
    long localTarget0 = target[0];
    long localTarget1 = target[1];
    portEXIT_CRITICAL(&mux);

    // --- 3. PID computation (one controller per motor) -------------------
    int pwr[NMOTORS];
    int dir[NMOTORS];

    pid[0].evalu(localPos0, localTarget0, deltaT, pwr[0], dir[0]);
    pid[1].evalu(localPos1, localTarget1, deltaT, pwr[1], dir[1]);

    // --- 4. Cross-motor synchronisation correction -----------------------
    // Even with identical PID gains, mechanical differences (friction, load)
    // cause the two wheels to drift apart over time. This correction detects
    // which wheel is ahead and nudges their PWM outputs to equalise them.
    //
    // syncError > 0 → motor 0 is ahead → slow motor 0, speed up motor 1.
    // syncError < 0 → motor 1 is ahead → speed up motor 0, slow motor 1.
    //
    // Clamped to ±50 so a large positional gap at startup (before the PID
    // has settled) cannot drive either motor's correction so hard that it
    // silences the motor for the first several ticks.
    int   syncError = (int)constrain((long)(localPos0 - localPos1), -50L, 50L);
    float syncKp    = 0.8f;

    // Only apply sync when both motors are running in the same direction.
    // Applying it during a turn (opposite directions) would fight the intended
    // differential and corrupt the turning radius.
    if (dir[0] == dir[1]) {
      if (dir[0] == 1) {
        // Both going forward.
        pwr[0] -= (int)(syncError * syncKp);
        pwr[1] += (int)(syncError * syncKp);
      } else {
        // Both going in reverse — correction sign inverts.
        pwr[0] += (int)(syncError * syncKp);
        pwr[1] -= (int)(syncError * syncKp);
      }
    }

    // --- 5. Apply outputs ------------------------------------------------
    for (int k = 0; k < NMOTORS; k++) {
      pwr[k] = constrain(pwr[k], 0, 255); // Hard clamp to valid PWM range.
      setMotor(dir[k], pwr[k], pwm[k], in1[k], in2[k]);
    }

    // Yield for 10 ms before the next iteration (~100 Hz control loop).
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
