/**
 * @file VPlotter_Controller.ino
 * @brief Firmware for a Planar Cable-Driven Robot (V-Plotter)
 * * This system utilizes TMC2209 stepper drivers via UART for precise control
 * and StallGuard-based sensorless homing. It performs real-time inverse 
 * kinematics to translate Cartesian (X,Y) coordinates into required cable lengths.
 */

#include <AccelStepper.h>
#include <TMCStepper.h>
#include <Servo.h>

// ==========================================
// CONFIGURATION & MACHINE GEOMETRY
// ==========================================
const float REAL_WIDTH   = 368.0;   // Distance between motor shafts (mm)
const float START_Y      = 110.0;   // Y-axis offset for home position (mm)

// TMC2209 STALLGUARD TUNING
const int STALL_VALUE = 60;     // 0 = Hardest to Stall, 255 = Most Sensitive
const int HOME_SPEED  = 1600;   
const int HOME_ACCEL  = 800;    

// ==========================================
// PIN DEFINITIONS & HARDWARE MAPPING
// ==========================================
#define SERIAL_PORT Serial1
#define R_SENSE 0.11f

TMC2209Stepper driverX(&SERIAL_PORT, R_SENSE, 0); 
TMC2209Stepper driverY(&SERIAL_PORT, R_SENSE, 1); 

#define X_STEP_PIN 2
#define X_DIR_PIN  5
#define Y_STEP_PIN 3
#define Y_DIR_PIN  6
#define ENABLE_PIN 8
#define SERVO_PIN  11  
#define X_DIAG_PIN 9   // TMC2209 DIAG pin for right motor stall detection
#define Y_DIAG_PIN 10  // TMC2209 DIAG pin for left motor stall detection

// ==========================================
// OBJECT INSTANTIATION
// ==========================================
AccelStepper stepperL(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN); 
AccelStepper stepperR(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
Servo penServo;

// KINEMATIC CONSTANTS
const float stepsPerRev = 200.0; 
const float microsteps  = 16.0;   
const float pulleyTeeth = 20.0;
const float beltPitch   = 2.0;    
const float stepsPerMM  = (stepsPerRev * microsteps) / (pulleyTeeth * beltPitch);

// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void homeSensorless();
void drawSquare();
void penUp();
void penDown();
void moveTo(float x, float y);
void setupDriver(TMC2209Stepper &driver, String name);
bool runUntilStall(int speedL, int speedR, int diagPin);

// ==========================================
// MAIN SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);      
  SERIAL_PORT.begin(115200); 
  
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // Disable motors during boot
  
  pinMode(X_DIAG_PIN, INPUT); 
  pinMode(Y_DIAG_PIN, INPUT);
  
  penServo.attach(SERVO_PIN);
  penUp();

  Serial.println("\n--- V-PLOTTER INITIALIZING ---");
  delay(200); 
  setupDriver(driverX, "X (Right)");
  setupDriver(driverY, "Y (Left)");

  digitalWrite(ENABLE_PIN, LOW); // Enable motors
  
  stepperL.setMaxSpeed(1200); stepperL.setAcceleration(600);
  stepperR.setMaxSpeed(1200); stepperR.setAcceleration(600);

  Serial.println("System Ready. Send 'H' to Home, 'D' for Test Pattern.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'H' || c == 'h') homeSensorless();
    else if (c == 'D' || c == 'd') drawSquare();
  }
}

// ==========================================
// CORE CONTROL FUNCTIONS
// ==========================================

/**
 * @brief Translates target Cartesian (X,Y) coordinates into motor steps.
 * * Applies inverse kinematics for a planar cable robot. Calculates the 
 * hypotenuse (cable length) from each motor to the target end-effector position.
 */
void moveTo(float x, float y) {
  if (stepperL.maxSpeed() < 100) stepperL.setMaxSpeed(1200);

  float targetY = START_Y + y; 

  // Calculate Cartesian distance from each motor to the target (X-axis)
  float distFromLeft  = (REAL_WIDTH / 2.0) + x;
  float distFromRight = (REAL_WIDTH / 2.0) - x;
  
  // Inverse Kinematics: Pythagorean theorem to find required cable lengths
  float lenL = sqrt( sq(distFromLeft)  + sq(targetY) );
  float lenR = sqrt( sq(distFromRight) + sq(targetY) );

  // Convert mm to steps and command motors
  stepperL.moveTo(lenL * stepsPerMM);
  stepperR.moveTo(lenR * stepsPerMM);

  // Blocking call for synchronous movement
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    stepperL.run();
    stepperR.run();
  }
}
