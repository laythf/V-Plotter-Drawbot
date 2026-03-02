#include <AccelStepper.h>
#include <TMCStepper.h>
#include <Servo.h>

// ==========================================
//           CONFIGURATION
// ==========================================
const float REAL_WIDTH   = 368.0;   
const float START_Y      = 110.0;    

// HARDWARE TUNING
// 0 = Hardest to Stall, 255 = Most Sensitive.
// Raised to 128 to ensure we detect the wall and stop bumping!
const int STALL_VALUE = 60;     
const int HOME_SPEED  = 1600;   
const int HOME_ACCEL  = 800;    

// ==========================================
//               PIN DEFINITIONS
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
#define X_DIAG_PIN 9   
#define Y_DIAG_PIN 10  

// ==========================================
//               OBJECTS
// ==========================================
AccelStepper stepperL(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN); 
AccelStepper stepperR(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
Servo penServo;

float stepsPerRev = 200.0; 
float microsteps  = 16.0;   
float pulleyTeeth = 20.0;
float beltPitch   = 2.0;    
float stepsPerMM  = (stepsPerRev * microsteps) / (pulleyTeeth * beltPitch);

// ==========================================
//           FUNCTION PROTOTYPES
// ==========================================
void homeSensorless();
void drawSquare();
void penUp();
void penDown();
void moveTo(float x, float y);
void setupDriver(TMC2209Stepper &driver, String name);
bool runUntilStall(int speedL, int speedR, int diagPin);

void setup() {
  Serial.begin(115200);      
  SERIAL_PORT.begin(115200); 
  
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); 
  
  pinMode(X_DIAG_PIN, INPUT); 
  pinMode(Y_DIAG_PIN, INPUT);
  
  penServo.attach(SERVO_PIN);
  penUp();

  Serial.println("\n--- SYSTEM STARTUP ---");
  delay(200); 
  setupDriver(driverX, "X (Right)");
  setupDriver(driverY, "Y (Left)");

  digitalWrite(ENABLE_PIN, LOW); 
  
  stepperL.setMaxSpeed(1200); stepperL.setAcceleration(600);
  stepperR.setMaxSpeed(1200); stepperR.setAcceleration(600);

  Serial.println("System Ready. Send 'H' to Home.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'H' || c == 'h') homeSensorless();
    else if (c == 'D' || c == 'd') drawSquare();
  }
}

void setupDriver(TMC2209Stepper &driver, String name) {
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(1000); 
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF); 
  driver.TPWMTHRS(0);        
  driver.SGTHRS(STALL_VALUE);
}

// Returns TRUE if stalled, FALSE if timed out
bool runUntilStall(int speedL, int speedR, int diagPin) {
  driverX.SG_RESULT(); driverY.SG_RESULT(); 
  
  stepperL.setSpeed(speedL);
  stepperR.setSpeed(speedR);
  
  // 1. Acceleration Phase (Ignore stalls for 200ms)
  unsigned long start = millis();
  while (millis() - start < 200) { stepperL.runSpeed(); stepperR.runSpeed(); }
  
  // 2. Monitoring Phase
  Serial.print("  Moving... ");
  
  // SAFETY TIMEOUT: 6 Seconds max. 
  // If it takes longer than 6s to find a wall, something is wrong.
  unsigned long timeout = millis();
  while (digitalRead(diagPin) == LOW) { 
    stepperL.runSpeed(); 
    stepperR.runSpeed(); 
    
    if (millis() - timeout > 6000) {
      Serial.println("TIMEOUT! (Stall not detected)");
      stepperL.stop(); stepperR.stop();
      return false; // Failed
    }
  }
  
  Serial.println("STALL DETECTED.");
  return true; // Success
}

void homeSensorless() {
  Serial.println("\n--- HOMING ---");
  stepperL.setMaxSpeed(2000); stepperL.setAcceleration(HOME_ACCEL);
  stepperR.setMaxSpeed(2000); stepperR.setAcceleration(HOME_ACCEL);

  // 1. Find Left
  Serial.println("Step 1: Find Left Wall");
  bool hitLeft = runUntilStall(-HOME_SPEED, HOME_SPEED, Y_DIAG_PIN);
  
  if (!hitLeft) {
    Serial.println("ERROR: Left Homing Timed Out. Increase sensitivity?");
    return;
  }
  
  stepperL.setCurrentPosition(0); stepperR.setCurrentPosition(0);
  // Back off
  stepperL.runToNewPosition(5 * stepsPerMM); 
  stepperR.runToNewPosition(-5 * stepsPerMM); 
  delay(200);

  // 2. Find Right
  Serial.println("Step 2: Find Right Wall");
  bool hitRight = runUntilStall(HOME_SPEED, -HOME_SPEED, X_DIAG_PIN);

  if (!hitRight) {
     Serial.println("ERROR: Right Homing Timed Out.");
     return;
  }
  
  long measuredSteps = stepperL.currentPosition();
  float measuredMM = measuredSteps / stepsPerMM;
  
  Serial.print("  Sensor Measured Width: "); Serial.println(measuredMM);
  
  // SANITY CHECK: If measurement is wildly wrong, STOP.
  if (measuredMM < 250.0 || measuredMM > 450.0) {
    Serial.println("  CRITICAL ERROR: Measured width is invalid.");
    Serial.println("  Check belts or Stall Sensitivity.");
    return;
  }

  // 3. APPLY CORRECT GEOMETRY
  Serial.println("  Applying Trusted Geometry...");
  stepperL.setCurrentPosition(REAL_WIDTH * stepsPerMM);
  stepperR.setCurrentPosition(0);
  
  // 4. CENTER
  long targetCenter = (REAL_WIDTH / 2.0) * stepsPerMM;
  Serial.println("Step 3: Centering...");
  stepperL.moveTo(targetCenter); 
  stepperR.moveTo(targetCenter); 
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    stepperL.run();
    stepperR.run();
  }

  // 5. SET START POINT
  stepperL.setCurrentPosition(targetCenter);
  stepperR.setCurrentPosition(targetCenter);
  
  Serial.println("Step 4: Moving to Start Position...");
  moveTo(0, 0); 
  
  Serial.println("Homing Done.");
}

void moveTo(float x, float y) {
  // FAST, SMOOTH MOVEMENT (No segmentation)
  if (stepperL.maxSpeed() < 100) stepperL.setMaxSpeed(1200);

  float targetY = START_Y + y; 

  float distFromLeft  = (REAL_WIDTH / 2.0) + x;
  float distFromRight = (REAL_WIDTH / 2.0) - x;
  
  float lenL = sqrt( sq(distFromLeft)  + sq(targetY) );
  float lenR = sqrt( sq(distFromRight) + sq(targetY) );

  stepperL.moveTo(lenL * stepsPerMM);
  stepperR.moveTo(lenR * stepsPerMM);

  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    stepperL.run();
    stepperR.run();
  }
}

void drawSquare() {
  Serial.println("Drawing Square...");
  // Scaled down slightly to 40mm to check geometry
  moveTo(-20, 0); penDown();          
  moveTo(20,  0); 
  moveTo(20,  40);     
  moveTo(-20, 40); 
  moveTo(-20, 0);     
  penUp(); 
  moveTo(0, 0); 
}

void penUp() { penServo.write(90); delay(200); }
void penDown() { penServo.write(20); delay(200); }