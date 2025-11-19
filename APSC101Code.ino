#include "AFMotor_R4.h"  // im guessing this is the library we need for the motor controller

// pin assignments 
const int turbiditySensor1Pin = A0;  // Dirty water tank
const int turbiditySensor2Pin = A1;  // Clean water tank
const int startButtonPin = 20;         // Digital pin for start
const int emergencyStopPin = 21;       // Must be pin 2 or 3 for interrupt
//const int dirtyPump = 1;              // Dirty water pump relay
const int pump2Pin = A8;              // Alum slurry pump relay
//const int cleanPump = 6;              // Clean water pump relay
const int greenLED = 49;
const int yellowLED = 48;
const int redLED = 47;

// Timing Parameters (in milliseconds - adjust based on testing)
const unsigned long turbidityReadTime = 60000;    // 1 minute initial reading
const unsigned long dirtyPumpRunTime = 60000;       // 30 seconds dirty water
const unsigned long pump2RunTime = 15000;        // 5 seconds alum
const unsigned long cleanPumpRunTime = 95000;       // 40 seconds clean transfer
const unsigned long fastMixTime = 30000;         // 15 seconds fast mixing
const unsigned long slowMixTime = 30000;         // 15 seconds slow mixing
const unsigned long settleTime = 75000;           // 45 seconds settlement
const unsigned long pressLowerTime = 8500;      // 30 seconds press operation
const unsigned long pressRaiseTime = 8800;      // 15 seconds to raise

// adjust values fro our motors (in between 0 and 255)
const int dirtyPumpSpeed = 200;
const int cleanPumpSpeed = 255;
const int fastMixSpeed = 255;   
const int slowMixSpeed = 150;    
const int pressSpeed = 120;       // for press motor

// Turbidity Thresholds
const float minTurbidityToTreat = 100.0;   // NTU to start treatment
const float minImprovement = 200.0;          // Minimum NTU improvement for success

const bool DEBUG = false;

// Motor Shield Setup (adjust motor numbers based on our connections)
// i went off guessing the structure of the motor controller, might need change
AF_DCMotor cleanPump(3);
AF_DCMotor dirtyPump(2);
AF_DCMotor mixingMotor(4);   // Motor 1 on shield for mixing
AF_DCMotor pressMotor(1);    // Motor 2 on shield for press

// System State Variables
enum SystemState {
  IDLE,
  PHASE_1_DISPENSING,
  PHASE_2_MIXING,
  PHASE_3_TRANSFER,
  EMERGENCY_STOPPED
};

SystemState currentState = IDLE; //currentstate is our variable that has the system states in it
unsigned long phaseStartTime = 0;
unsigned long currentStepStartTime = 0;
int currentStep = 0;

// measurement variables
float initialTurbidity = 0.0;
float finalTurbidity = 0.0;
float turbiditySum = 0.0;
int turbidityReadCount = 0;
unsigned long lastTurbidityRead = 0;

// emergency stop flag
volatile bool emergencyStop = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Water Treatment System Starting");

  
  // initialize pins
  pinMode(startButtonPin, INPUT_PULLUP); // input pullup makes the default state high and it goes low when pressed
  pinMode(emergencyStopPin, INPUT_PULLUP); // when wiring buttons, it has to be a bridge between gnd and the pin, so the (pin - button - gnd) is the wiring
  
  pinMode(pump2Pin, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  // ensure all pumps start off
  
  digitalWrite(pump2Pin, LOW);
  digitalWrite(greenLED, HIGH);
  digitalWrite(yellowLED, LOW);
  digitalWrite(redLED, LOW);
  
  // initialize motors at zero speed
  cleanPump.setSpeed(0);
  cleanPump.run(RELEASE);
  dirtyPump.setSpeed(0);
  dirtyPump.run(RELEASE);
  mixingMotor.setSpeed(0);
  mixingMotor.run(RELEASE);
  pressMotor.setSpeed(0);
  pressMotor.run(RELEASE);
  
  // Attach interrupt for emergency stop 
  // this function will monitoe emergency stop pin and if its activated, it will execute the duntion emergencystop
  // FALLING states that the stop is triggered when state goes from high to low
  // digital pin to unterrupt functoion will translate pin number to according system interruption pin number
  // attachInterrupt(digitalPinToInterrupt(emergencyStopPin), setEmergencyStop, FALLING);
  
  Serial.println("System Initialized, Press START button to begin");
  Serial.println("EMERGENCY STOP available at any time\n");
}


void loop() {
  // Check for emergency stop
  if (emergencyStop) {
    handleEmergencyStop();
    return;
  }

  if (digitalRead(emergencyStopPin) == LOW) {
    delay(50); // this delay is here to make sure the button is really pressed and is not having a debounce
    if (digitalRead(emergencyStopPin) == LOW) {
      setEmergencyStop();
      return;
    }
  }
  
  // State machine
  switch (currentState) {
    case IDLE:
      handleIdleState();
      break;
      
    case PHASE_1_DISPENSING:
      handlePhase1();
      break;
      
    case PHASE_2_MIXING:
      handlePhase2();
      break;
      
    case PHASE_3_TRANSFER:
      handlePhase3();
      break;
      
    case EMERGENCY_STOPPED:
      //the system will need a manual reset because every pin is deactivated now
      break;
  }
}

// handlers... 
void handleIdleState() {
  if (digitalRead(startButtonPin) == LOW) {
    delay(50); // this delay is here to make sure the button is really pressed and is not having a debounce
    if (digitalRead(startButtonPin) == LOW) {
      digitalWrite(greenLED, LOW);
      digitalWrite(yellowLED, HIGH);

      Serial.println("\nSTARTING TREATMENT CYCLE");
      Serial.println("Time: 0:00");
      currentState = PHASE_1_DISPENSING;
      phaseStartTime = millis();
      currentStepStartTime = millis(); // this represents the time at when the start button was pressed
      currentStep = 0;
      turbiditySum = 0;
      turbidityReadCount = 0;
    }
  }
}

void handlePhase1() {
  unsigned long elapsedTime = millis() - currentStepStartTime; //this will return the time since the start button has been pressed
  
  switch(currentStep) {
    case 0: // Read turbidity for 1 minute
      if (elapsedTime == 0) {
        Serial.println("\n PHASE 1: SOLUTION DISPENSING");
        Serial.println("Stage 1: Reading Initial Turbidity (1 minute)...");
      }

      // Read turbidity every 100ms
      if (millis() - lastTurbidityRead >= 100) {
        float voltage = analogRead(turbiditySensor1Pin) * (5.0 / 1023.0); // the last part is to calculate and translate analog value into voltages
        float turbidity = calculateTurbidity(voltage);
        turbiditySum += turbidity;
        turbidityReadCount++;
        lastTurbidityRead = millis();

        // Print progress every 5 seconds
        if (elapsedTime % 5000 < 100) {
          Serial.print("  Turbidity reading at ");
          Serial.print(elapsedTime / 1000);
          Serial.print("s: ");
          Serial.print(turbidity);
          Serial.print(" NTU");
          Serial.print(" (");
          Serial.print(voltage);
          Serial.println(" V)");
        }
      }
      
      if (elapsedTime >= turbidityReadTime) {
        initialTurbidity = turbiditySum / turbidityReadCount; // this averages all calculated turbidities and divides them with each cacluilation to find the average ntu 
        Serial.print("Initial Turbidity Average: ");
        Serial.print(initialTurbidity);
        Serial.println(" NTU");
        
        if (initialTurbidity < minTurbidityToTreat && !DEBUG) {
          Serial.println("Water already clean! Turbidity < 100 NTU");
          Serial.println("Treatment not required, Ending cycle");
          resetSystem();
        } else { // else runs when water is not clean
          currentStep = 1; // next step starts
          currentStepStartTime = millis();
        }
      }
      break;
      
    case 1: // Pump dirty water
      if (elapsedTime == 0) {
        Serial.println("\nStage 2: Pumping dirty water to coagulation tank...");
        dirtyPump.setSpeed(dirtyPumpSpeed);
        dirtyPump.run(FORWARD);
      }
      if (elapsedTime >= dirtyPumpRunTime) {
        dirtyPump.run(RELEASE);
        Serial.println("  Dirty water transfer complete");
        currentStep = 2;
        currentStepStartTime = millis();
      }
      break;
      
    case 2: // Pump alum slurry
      if (elapsedTime == 0) {
        Serial.println("Stage 3: Adding alum slurry...");
        digitalWrite(pump2Pin, HIGH);
      }
      if (elapsedTime >= pump2RunTime) { // after pump time has passes
        digitalWrite(pump2Pin, LOW);
        Serial.println("  Alum addition complete");
        currentStep = 3;
        currentStepStartTime = millis();
      }
      break;
      
    case 3: // Wait 2 seconds
      if (elapsedTime >= 2000) {
        Serial.println("Phase 1 Complete!\n");
        currentState = PHASE_2_MIXING;
        currentStep = 0;
        currentStepStartTime = millis();
      }
      break;
  }
}

void handlePhase2() {
  unsigned long elapsedTime = millis() - currentStepStartTime;
  
  switch(currentStep) {
    case 0: // Fast mixing
      if (elapsedTime == 0) {
        Serial.println("PHASE 2: MIXING & FILTRATION ");
        Serial.println("Stage 1: Fast Mixing...");
        mixingMotor.setSpeed(slowMixSpeed);
        mixingMotor.run(FORWARD);
        

      }
      if (elapsedTime >= fastMixTime) {
        Serial.println("  Fast mixing complete");
        currentStep = 1;
        currentStepStartTime = millis();
      }
      break;
      
    case 1: // Slow mixing
      if (elapsedTime == 0) {
        Serial.println("Stage 2: Slow Mixing...");
        mixingMotor.setSpeed(slowMixSpeed);
      }
      if (elapsedTime >= slowMixTime) {
        mixingMotor.run(RELEASE);
        Serial.println("  Slow mixing complete");
        currentStep = 2;
        currentStepStartTime = millis();
      }
      break;
      
    case 2: // Settlement
      if (elapsedTime == 0) {
        Serial.println("Stage 3: Settlement period...");
      }
      if (elapsedTime >= settleTime) {
        Serial.println("  Settlement complete");
        currentStep = 3;
        currentStepStartTime = millis();
      }
      break;
      
    case 3: // Lower press
      if (elapsedTime == 0) {
        Serial.println("Stage 4: Lowering filter press...");
        pressMotor.setSpeed(pressSpeed);
        pressMotor.run(BACKWARD);
      }
      if (elapsedTime >= pressLowerTime) {
        pressMotor.run(RELEASE);
        Serial.println("  Press lowered - filtration complete");
        Serial.println("Phase 2 Complete!\n");
        currentState = PHASE_3_TRANSFER;
        currentStep = 0;
        currentStepStartTime = millis();
      }
      break;
  }
}

void handlePhase3() {
  unsigned long elapsedTime = millis() - currentStepStartTime;
  
  switch(currentStep) {
    case 0: // Transfer clean water
      if (elapsedTime == 0) {
        Serial.println("PHASE 3: CLEAN WATER TRANSFER ");
        Serial.println("Stage 1: Transferring treated water...");
        cleanPump.setSpeed(cleanPumpSpeed);
        cleanPump.run(FORWARD);
      }
      if (elapsedTime >= cleanPumpRunTime) {
        cleanPump.run(RELEASE);
        Serial.println("  Water transfer complete");
        currentStep = 1;
        currentStepStartTime = millis();
      }
      break;
      
    case 1: // Read final turbidity
      if (elapsedTime == 0) {
        Serial.println("Stage 2: Reading final turbidity...");
        //reset turbidity readings
        turbiditySum = 0;
        turbidityReadCount = 0;
      }

      // Read turbidity every 100ms
      if (millis() - lastTurbidityRead >= 100) {
        float voltage = analogRead(turbiditySensor2Pin) * (5.0 / 1023.0); // the last part is to calculate and translate analog value into voltages
        float turbidity = calculateTurbidity(voltage);
        turbiditySum += turbidity;
        turbidityReadCount++;
        lastTurbidityRead = millis();

        // Print progress every 5 seconds
        if (elapsedTime % 5000 < 100) {
          Serial.print("  Turbidity reading at ");
          Serial.print(elapsedTime / 1000);
          Serial.print("s: ");
          Serial.print(turbidity);
          Serial.print(" NTU");
          Serial.print(" (");
          Serial.print(voltage);
          Serial.println(" V)");
        }
      }
      
      if (elapsedTime >= turbidityReadTime) {
        finalTurbidity = turbiditySum / turbidityReadCount; // this averages all calculated turbidities and divides them with each cacluilation to find the average ntu 
        Serial.print("Final Turbidity Average: ");
        Serial.print(finalTurbidity);
        Serial.println(" NTU");
        
        // if (finalTurbidity < minTurbidityToTreat && !DEBUG) {
        //   Serial.println("Water already clean! Turbidity < 100 NTU");
        //   Serial.println("Treatment not required, Ending cycle");
        //   resetSystem();
        // } else { // else runs when water is not clean
        // }

        Serial.print("\n========== TREATMENT RESULTS ==========");
        Serial.print("\nInitial Turbidity: ");
        Serial.print(initialTurbidity);
        Serial.println(" NTU");
        Serial.print("Final Turbidity: ");
        Serial.print(finalTurbidity);
        Serial.println(" NTU");
        Serial.print("Improvement: ");
        Serial.print(initialTurbidity - finalTurbidity);
        Serial.println(" NTU");
        
        float totalTime = (millis() - phaseStartTime) / 60000.0;
        Serial.print("Total Time: ");
        Serial.print(totalTime);
        Serial.println(" minutes");
        
        if (initialTurbidity - finalTurbidity >= minImprovement) {
          Serial.println("\nSUCCESS! Treatment achieved >200 NTU improvement");
        } else {
          Serial.println("\nWARNING: Treatment insufficient (<200 NTU improvement)");
        }
        
        currentStep = 2;
        currentStepStartTime = millis();
      }


      
      // if (elapsedTime >= 2000) { // Wait 2 seconds for stable reading
      //   float voltage = analogRead(turbiditySensor2Pin) * (5.0 / 1023.0);
      //   finalTurbidity = calculateTurbidity(voltage);
        
        
      // }
      break;
      
    case 2: // Raise press
      if (elapsedTime == 0) {
        Serial.println("\nResetting press position...");
        pressMotor.setSpeed(pressSpeed);
        pressMotor.run(FORWARD);
      }
      if (elapsedTime >= pressRaiseTime) {
        pressMotor.run(RELEASE);
        Serial.println("System reset complete");
        Serial.println("\nCYCLE COMPLETE\n");
        resetSystem();
      }
      break;
  }
}

// helper functions
float calculateTurbidity(float voltage) {
  // Formula from project document
  return -1120.4 * voltage * voltage + 5742.3 * voltage - 4352.9;
}

void resetSystem() {
  currentState = IDLE;
  currentStep = 0;
  dirtyPump.run(RELEASE);
  digitalWrite(pump2Pin, LOW);
  cleanPump.run(RELEASE);

  digitalWrite(greenLED, HIGH);
  digitalWrite(yellowLED, LOW);
  digitalWrite(redLED, LOW);

  mixingMotor.run(RELEASE);
  pressMotor.run(RELEASE);
  Serial.println("System ready for next cycle - Press START to begin");
}

void setEmergencyStop() {
  emergencyStop = true;
}

void handleEmergencyStop() {
  // Stop everything immediately
  dirtyPump.run(RELEASE);
  digitalWrite(pump2Pin, LOW);
  cleanPump.run(RELEASE);
  mixingMotor.run(RELEASE);
  pressMotor.run(RELEASE);
  
  //Update LED status
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(yellowLED, LOW);

  currentState = EMERGENCY_STOPPED;
  
  Serial.println("\n!!! EMERGENCY STOP ACTIVATED !!!");
  Serial.println("All systems halted");
  Serial.println("Manual reset required - restart Arduino to continue");
  
  // Disable further operations
  while(1) {
    delay(1000);
  }
}