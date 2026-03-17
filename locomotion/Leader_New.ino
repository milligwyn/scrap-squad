// =============================================================================
// Capstone Project - Wall-ECE
// File:    Leader_New.ino
// Author:  Dexter Fason
// Board:   Arduino Giga R1 Wi-Fi
//
// Original architecture credit:
//   Curio Res on YT - https://youtube.com/@curiores111?si=Edl755O9VMZwGuMm
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// This file runs on the Arduino Giga R1, which has two processor cores:
//
//   M7 Core  ("The Brain")  — High-level mission logic. Decides WHAT the robot
//                             does and WHEN. Communicates with M4 via RPC.
//
//   M4 Core  ("The Brawns") — Low-level locomotion controller. Manages the
//                             Finite State Machine (FSM) that steps through a
//                             path, sends encoder targets to the two Follower
//                             boards over I2C, and reads back their positions
//                             to know when each step is complete.
//
// The two cores share a single .ino file. The preprocessor macros CORE_CM4
// and CORE_CM7 gate which sections each core compiles and runs.
//
// -----------------------------------------------------------------------------
// HOW MOVEMENT WORKS (big picture)
// -----------------------------------------------------------------------------
//  1. M7 calls RPC.call("executePath", pathID) to tell M4 which pre-defined
//     path to execute (e.g. drive to the pedestal).
//
//  2. M4's FSM steps through each PathCommand in the chosen path array.
//     For each step it:
//       a. Adds the step's pulse distance to the running target counters.
//       b. Sends the new targets to Follower 1 (front wheels, I2C addr 1)
//          and Follower 2 (rear wheels, I2C addr 2).
//       c. Polls the followers' encoder positions until all four wheels are
//          within ERROR_MARGIN pulses of their targets.
//       d. Waits for the step's delayAfter period before moving to the next.
//
//  3. Each Follower runs its own PID controller locally to drive its two
//     motors to the target position. M4 only monitors progress — it does
//     not send PWM signals directly.
//
//  4. When the full path is complete, M4 sets isMoving = false so M7 knows
//     it can proceed with the next mission action (e.g. arm control).
//
// -----------------------------------------------------------------------------
// PULSE MATH
// -----------------------------------------------------------------------------
//  Motor gearbox ratio : 60:1
//  Encoder resolution  : 11 pulses per motor revolution (before gearbox)
//  Pulses per wheel rev: 11 * 60 = 660
//  Wheel circumference : ~0.319 m  (diameter ~0.1016 m, i.e. 4-inch wheel)
//  Pulses per meter    : 660 / 0.319 ≈ 660 * 3.132 ≈ 2067 pulses/m
// =============================================================================

#include <RPC.h>

// =============================================================================
// SHARED DEFINITIONS — visible to BOTH cores
// These constants and types are compiled into both M4 and M7.
// =============================================================================

// Number of encoder pulses the motor produces per full wheel revolution.
// 11 pulses/motor-rev × 60:1 gearbox = 660 pulses/wheel-rev.
const float PULSES_PER_TURN = 11.0 * 60;

// How many encoder pulses correspond to travelling exactly one metre.
// 3.132 = wheel revolutions needed to travel 1 m (1 / wheel circumference in m).
const float PULSES_PER_METER = PULSES_PER_TURN * 3.132;

// How close (in pulses) a wheel must be to its target before we consider it
// "arrived". 100 pulses ≈ 4.8 cm at this gear ratio — adjust during tuning.
const float ERROR_MARGIN = 100;

// -----------------------------------------------------------------------------
// PathCommand
// -----------------------------------------------------------------------------
// Describes a single movement step. A "path" is an array of these.
//
//   leftDist   — encoder pulse distance for the LEFT-side wheels (motors 0 & 2).
//                Positive = forward. Negative = reverse.
//   rightDist  — encoder pulse distance for the RIGHT-side wheels (motors 1 & 3).
//                Differential between left/right makes the robot turn.
//   delayAfter — milliseconds to pause AFTER this step completes before
//                executing the next one. Gives the robot time to settle.
struct PathCommand
{
  long leftDist;
  long rightDist;
  unsigned long delayAfter;
};

// -----------------------------------------------------------------------------
// Pre-defined paths
// -----------------------------------------------------------------------------
// Each array entry is one PathCommand step. The FSM on M4 walks through them
// in order. Targets are CUMULATIVE (absolute encoder counts), so distances
// here represent how far to travel from wherever the robot currently is.

// pathForward — drives straight forward exactly one metre, then stops for 1 s.
const PathCommand pathForward[] =
{
  { (long)PULSES_PER_METER, (long)PULSES_PER_METER, 1000 }
};

// pathToPedestal — multi-step sequence to approach the pedestal:
//   Step 0: Drive forward 2 m (long approach), pause 0.5 s.
//   Step 1: Drive forward 0.5 m (medium approach), pause 0.5 s.
//   Step 2: Drive forward 0.25 m (fine positioning), pause 2 s to settle.
// The progressive slow-down reduces overshoot as the robot closes in.
const PathCommand pathToPedestal[] =
{
  { (long)(PULSES_PER_METER * 2),   (long)(PULSES_PER_METER * 2),   500  },
  { (long)(PULSES_PER_METER / 2),   (long)(PULSES_PER_METER / 2),   500  },
  { (long)(PULSES_PER_METER / 4),   (long)(PULSES_PER_METER / 4),   2000 }
};


// =============================================================================
// M4 CORE — Locomotion Controller
// Everything inside this #ifdef only compiles and runs on the M4 processor.
// =============================================================================
#ifdef CORE_CM4
#include <Wire.h>

// target[0..3] — absolute encoder count each wheel should reach.
//   [0] = front-left   [1] = front-right   (Follower 1, I2C addr 1)
//   [2] = rear-left    [3] = rear-right    (Follower 2, I2C addr 2)
// Targets are additive: each path step increases them from their current value.
long target[4] = {0, 0, 0, 0};

// pos[4] — most recently read encoder counts from the two followers.
// Updated by readPositionsFromFollowers() before checking targetsReached().
long pos[4] = {0, 0, 0, 0};

// -----------------------------------------------------------------------------
// FSM State Definitions
// -----------------------------------------------------------------------------
// The M4 loop() runs a Finite State Machine with four states:
//
//   IDLE         — doing nothing; waiting for M7 to trigger a path via RPC.
//   SEND_COMMAND — load the current step's targets and transmit them to the
//                  followers over I2C.
//   MOVING       — poll follower positions each loop; stay here until all four
//                  wheels are within ERROR_MARGIN of their targets.
//   DELAYING     — step complete; wait delayAfter ms before the next step.
enum PathState
{
  IDLE,
  SEND_COMMAND,
  MOVING,
  DELAYING
};

PathState currentState = IDLE;   // FSM starts idle at power-on.

// Pointer to whichever path array is currently being executed.
const PathCommand* activePath;

// Total number of steps in the active path (computed at path start).
int activePathLength = 0;

// Index of the step currently being executed (0-based).
int currentStep = 0;

// Timestamp (ms) recorded when a step first enters the DELAYING state.
unsigned long delayStartTime = 0;

// True while a path is executing; M7 polls this via RPC to know when to
// proceed with the next high-level action (e.g. arm control).
bool isMoving = false;

// Forward declarations — needed because these functions call each other and
// are defined below setup()/loop().
void sendTargetsToFollowers();
void readPositionsFromFollowers();
bool targetsReached();
void stopMotors();

// -----------------------------------------------------------------------------
// executePath()
// -----------------------------------------------------------------------------
// Called by M7 via RPC to start a pre-defined path.
//
//   pathID 0 → pathForward
//   pathID 1 → pathToPedestal
//
// Returns 1 if the path was accepted and started.
// Returns 0 if the robot is already moving (M7 should wait and retry) or if
// the pathID is unrecognised.
int executePath(int pathID)
{
  // Reject new commands while a path is already running.
  if (isMoving) return 0;

  if (pathID == 0)
  {
    activePath       = pathForward;
    activePathLength = sizeof(pathForward) / sizeof(pathForward[0]);
  }
  else if (pathID == 1)
  {
    activePath       = pathToPedestal;
    activePathLength = sizeof(pathToPedestal) / sizeof(pathToPedestal[0]);
  }
  else
  {
    return 0; // Unknown path ID.
  }

  currentStep  = 0;
  isMoving     = true;
  currentState = SEND_COMMAND; // Kick off the FSM immediately.
  return 1;
}

// -----------------------------------------------------------------------------
// checkIfMoving()
// -----------------------------------------------------------------------------
// Called by M7 via RPC to poll whether the robot is still executing a path.
// M7 calls this in a loop (with a short delay) to block until locomotion is
// complete before doing anything else (e.g. triggering the arm).
bool checkIfMoving()
{
  return isMoving;
}

// -----------------------------------------------------------------------------
// setup() — M4
// -----------------------------------------------------------------------------
void setup()
{
  Wire.begin();    // Start as I2C master (no address argument = master mode).
  RPC.begin();     // Initialise inter-core communication.

  // Register the functions that M7 can call by name over RPC.
  RPC.bind("executePath",     executePath);
  RPC.bind("checkingIfMoving", checkIfMoving);
}

// -----------------------------------------------------------------------------
// loop() — M4  (runs the FSM on every iteration)
// -----------------------------------------------------------------------------
void loop()
{
  switch (currentState)
  {
    // -------------------------------------------------------------------------
    case IDLE:
      // Nothing to do — M7 will call executePath() via RPC when it is ready.
    break;

    // -------------------------------------------------------------------------
    case SEND_COMMAND:
      // Add the current step's distances to the running totals.
      // Because targets are absolute encoder counts, we always ADD rather than
      // SET — this way the robot travels incrementally from its current position.
      target[0] += activePath[currentStep].leftDist;
      target[1] += activePath[currentStep].rightDist;
      target[2] += activePath[currentStep].leftDist;
      target[3] += activePath[currentStep].rightDist;

      // Transmit the new targets to both follower boards over I2C.
      sendTargetsToFollowers();
      currentState = MOVING;
    break;

    // -------------------------------------------------------------------------
    case MOVING:
      // Keep polling until every wheel reaches its target (within ERROR_MARGIN).
      if (targetsReached())
      {
        stopMotors();                    // Lock targets to current position.
        delayStartTime = millis();       // Record when we entered DELAYING.
        currentState   = DELAYING;
      }
    break;

    // -------------------------------------------------------------------------
    case DELAYING:
      // Non-blocking wait: compare elapsed time against the step's delayAfter.
      // Using millis() avoids blocking the processor during the pause.
      if (millis() - delayStartTime >= activePath[currentStep].delayAfter)
      {
        currentStep++;

        if (currentStep >= activePathLength)
        {
          // All steps complete — signal M7 that locomotion is done.
          isMoving     = false;
          currentState = IDLE;
        }
        else
        {
          // More steps remain — load and send the next one.
          currentState = SEND_COMMAND;
        }
      }
    break;
  }
}

// -----------------------------------------------------------------------------
// stopMotors()
// -----------------------------------------------------------------------------
// Halts all four wheels by reading the current encoder positions and setting
// those positions as the new targets. This causes each follower's PID to see
// zero error and output zero power — a clean, position-locked stop.
void stopMotors()
{
  readPositionsFromFollowers();
  for (int i = 0; i < 4; i++)
  {
    target[i] = pos[i]; // PID error = target - pos = 0, so motor output → 0.
  }
  sendTargetsToFollowers();
}

// -----------------------------------------------------------------------------
// targetsReached()
// -----------------------------------------------------------------------------
// Returns true only when ALL four wheels are within ERROR_MARGIN encoder pulses
// of their respective targets. A single wheel being out of tolerance returns
// false — the robot waits until every wheel is settled.
bool targetsReached()
{
  readPositionsFromFollowers();
  for (int i = 0; i < 4; i++)
  {
    if (abs(target[i] - pos[i]) > ERROR_MARGIN)
    {
      return false;
    }
  }
  return true;
}

// -----------------------------------------------------------------------------
// sendTargetsToFollowers()
// -----------------------------------------------------------------------------
// Sends the current target encoder counts to both follower boards over I2C.
//
//   Follower 1 (addr 1) receives target[0] (front-left) and target[1] (front-right).
//   Follower 2 (addr 2) receives target[2] (rear-left)  and target[3] (rear-right).
//
// Each 'long' is 4 bytes, so each transmission is 8 bytes total.
// Wire.write() sends raw bytes; the follower's receiveEvent() reassembles them.
void sendTargetsToFollowers()
{
  Wire.beginTransmission(1);
  Wire.write((byte*)&target[0], sizeof(long) * 2); // Send target[0] and target[1].
  Wire.endTransmission();

  Wire.beginTransmission(2);
  Wire.write((byte*)&target[2], sizeof(long) * 2); // Send target[2] and target[3].
  Wire.endTransmission();
}

// -----------------------------------------------------------------------------
// readPositionsFromFollowers()
// -----------------------------------------------------------------------------
// Requests the current encoder positions from both follower boards over I2C
// and stores them in pos[]. Called by targetsReached() and stopMotors().
//
//   Follower 1 (addr 1) returns pos[0] (front-left) and pos[1] (front-right).
//   Follower 2 (addr 2) returns pos[2] (rear-left)  and pos[3] (rear-right).
//
// Wire.requestFrom() blocks until the bytes arrive or a timeout occurs.
// Wire.readBytes() reads the raw bytes back into the pos[] array directly.
void readPositionsFromFollowers()
{
  // Read from Follower 1 (front wheels).
  Wire.requestFrom(1, (int)(sizeof(long) * 2));
  if (Wire.available() >= (int)(sizeof(long) * 2))
  {
    Wire.readBytes((char*)&pos[0], sizeof(long) * 2);
  }

  // Read from Follower 2 (rear wheels).
  Wire.requestFrom(2, (int)(sizeof(long) * 2));
  if (Wire.available() >= (int)(sizeof(long) * 2))
  {
    Wire.readBytes((char*)&pos[2], sizeof(long) * 2);
  }
}

#endif // END CORE_CM4


// =============================================================================
// M7 CORE — Mission Brain
// Everything inside this #ifdef only compiles and runs on the M7 processor.
// =============================================================================
#ifdef CORE_CM7

// Prevents loop() from re-triggering the mission after it completes.
// Set to true once the robot has arrived and the arm sequence finishes.
// Replace this with a proper M7-side FSM when more mission phases are added.
bool missionDone = false;

// -----------------------------------------------------------------------------
// setup() — M7
// -----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial);   // Wait for Serial Monitor to connect (USB CDC).
  RPC.begin();       // Initialise inter-core communication with M4.
  delay(1000);       // Brief pause to allow M4 to finish its own setup().
  Serial.println("Wall-ECE M7 Core Initialized.");
}

// -----------------------------------------------------------------------------
// loop() — M7
// -----------------------------------------------------------------------------
// Current mission sequence (proof-of-concept):
//   1. Tell M4 to execute pathToPedestal (pathID = 1).
//   2. Block-poll until M4 reports movement is complete.
//   3. Trigger arm/stacking logic (placeholder delay for now).
//   4. Set missionDone to prevent this sequence from repeating.
//
// When the sensing and stacking subsystems are integrated, this loop will be
// replaced by a full M7-side FSM with states like:
//   NAVIGATING -> SENSING -> STACKING -> COMPLETE
// -----------------------------------------------------------------------------
void loop()
{
  if (missionDone)
  {
    // Mission is complete — hold here. Add next mission phase when ready.
    delay(1000);
    return;
  }

  Serial.println("M7: Instructing M4 to drive to pedestal...");

  // Call executePath(1) on M4 via RPC. Returns 1 if accepted, 0 if busy.
  auto result = RPC.call("executePath", 1).as<int>();

  if (result == 1)
  {
    Serial.println("M7: Waiting for movement to finish...");

    // Poll M4 every 50 ms until it reports isMoving == false.
    // This is a blocking wait on M7 only — M4 continues running its FSM
    // independently on its own core throughout this loop.
    while (RPC.call("checkingIfMoving").as<bool>() == true)
    {
      delay(50);
    }

    Serial.println("M7: Robot has arrived at the pedestal! Executing arm logic...");

    // --- Arm / stacking logic will be inserted here ---
    // For now, a 10-second placeholder simulates the arm sequence duration.
    delay(10000);

    missionDone = true;
  }
}

#endif // END CORE_CM7
