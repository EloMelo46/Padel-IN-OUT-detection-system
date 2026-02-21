// -----------------------------------------------------------
// Pins
// -----------------------------------------------------------
int greenLED_ext = 53;  // Outer green LED for "IN"
int redLED_ext   = 51;  // Outer red LED for "OUT"
int greenLED_int = 52;  // Inner green LED for "IN"
int redLED_int   = 50;  // Inner red LED for "OUT"

// 3 Floor sensors
int floorPins[3]    = {A0, A1, A2};     
int floorValues[3]  = {0};  

// Control LEDs for Floor sensors
int controlLEDs[3]  = {49, 47, 45};

// New pins for the three FSR control LEDs
int fsrControlLEDs[3] = {48, 46, 44};

// New pin for the FSR sensor (Wall sensor)
int fsrSensorPin = A6;

// -----------------------------------------------------------
// Global variables
// -----------------------------------------------------------
int   fsrValue = 0;      // Raw FSR value
float U_adc    = 0.0;    // Voltage in mV

// Timestamps Floor/FSR
unsigned long firstMicrosFloor = 0;
unsigned long firstMicrosFSR   = 0;
unsigned long startWaitMicros  = 0;  

// -----------------------------------------------------------
// Time constants (in microseconds)
// -----------------------------------------------------------
const unsigned long interval      = 2000000;  // LED on-time (2s)
const unsigned long triggerWindow = 500000;  // Time window (0.5s) for the 2nd trigger
const unsigned long resetWindow   = 1000000;  // 1s to "forget"

// -----------------------------------------------------------
// LED status variables
// -----------------------------------------------------------
bool isGreenActive = false;
bool isRedActive   = false;

// -----------------------------------------------------------
// Floor-2 and Floor-3 deactivation logic
// -----------------------------------------------------------
unsigned long floor2BlockedSince = 0;
bool floor2Disabled = false;

unsigned long floor3BlockedSince = 0;
bool floor3Disabled = false;

// -----------------------------------------------------------
// FSM states
// -----------------------------------------------------------
enum DetectionState {
  IDLE,
  WAITING_FOR_FSR,   // Floor triggered first
  WAITING_FOR_FLOOR  // FSR triggered first
};

DetectionState currentState = IDLE;

// Previous sensor states (edge detection)
static bool floorInterruptedOld = false;
static bool fsrInterruptedOld   = false;

// -----------------------------------------------------------
// Setup
// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Main LEDs
  pinMode(greenLED_ext, OUTPUT);
  pinMode(redLED_ext,   OUTPUT);
  pinMode(greenLED_int, OUTPUT);
  pinMode(redLED_int,   OUTPUT);

  digitalWrite(greenLED_ext, LOW);
  digitalWrite(redLED_ext,   LOW);
  digitalWrite(greenLED_int, LOW);
  digitalWrite(redLED_int,   LOW);

  // Floor control LED pins
  for (int i = 0; i < 3; i++) {
    pinMode(controlLEDs[i], OUTPUT);
    digitalWrite(controlLEDs[i], LOW);
  }

  // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  // FSR control LED pins (set as OUTPUT and initially off)
  for (int i = 0; i < 3; i++) {
    pinMode(fsrControlLEDs[i], OUTPUT);
    digitalWrite(fsrControlLEDs[i], LOW);
  }
  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  // Read Floor sensors once
  for (int i = 0; i < 3; i++) {
    floorValues[i] = analogRead(floorPins[i]);
  }

  // Initial FSR read
  fsrValue = analogRead(fsrSensorPin);
  U_adc    = (fsrValue / 1023.0) * 5000.0;

  // Determine Floor/FSR status at startup
  bool floorInterrupted = (
       floorValues[0] > 100
    || (floorValues[1] > 100 && !floor2Disabled)
    || (floorValues[2] > 100 && !floor3Disabled)
  );
  bool fsrInterrupted = (U_adc < 4000.0); // Threshold in mV

  unsigned long nowMicros = micros();

  if (fsrInterrupted && !floorInterrupted) {
    currentState    = WAITING_FOR_FLOOR;
    firstMicrosFSR  = nowMicros;
    startWaitMicros = nowMicros;
    Serial.println("Setup: FSR already triggered => WAITING_FOR_FLOOR");
  }
  else if (floorInterrupted && !fsrInterrupted) {
    currentState      = WAITING_FOR_FSR;
    firstMicrosFloor  = nowMicros;
    startWaitMicros   = nowMicros;
    Serial.println("Setup: Floor already triggered => WAITING_FOR_FSR");
  }
  else {
    currentState = IDLE;
    Serial.println("Setup: IDLE");
  }
}

// -----------------------------------------------------------
// Helper functions
// -----------------------------------------------------------
void triggerRed() {
  Serial.println("=> RED action executed");
  
  // If green is still active, turn it off
  if (isGreenActive) {
    digitalWrite(greenLED_ext, LOW);
    digitalWrite(greenLED_int, LOW);
    isGreenActive = false;
    Serial.println("Red overrides green.");
  }

  // Red blinking (8×)
  for (int i = 0; i < 8; i++) {
    digitalWrite(redLED_ext, HIGH);
    digitalWrite(redLED_int, HIGH);
    delay(50);
    digitalWrite(redLED_ext, LOW);
    digitalWrite(redLED_int, LOW);
    delay(50);
  }

  // 3s continuous light
  digitalWrite(redLED_ext, HIGH);
  digitalWrite(redLED_int, HIGH);
  delay(3000);
  digitalWrite(redLED_ext, LOW);
  digitalWrite(redLED_int, LOW);

  isRedActive = false;
}

void triggerGreen(unsigned long nowMicros) {
  Serial.println("=> GREEN action");
  digitalWrite(greenLED_ext, HIGH);
  digitalWrite(greenLED_int, HIGH);
  isGreenActive = true;
}

// -----------------------------------------------------------
// Main (loop)
// -----------------------------------------------------------
void loop() {
  unsigned long currentMicros = micros();

  // ---------------------------------------------------------
  // (1) Read Floor sensors and update Floor control LEDs
  // ---------------------------------------------------------
  for (int i = 0; i < 3; i++) {
    floorValues[i] = analogRead(floorPins[i]);

    // LED ON if sensor value <= 100 => "free"
    if (floorValues[i] <= 100) {
      digitalWrite(controlLEDs[i], HIGH);
    } else {
      digitalWrite(controlLEDs[i], LOW);
    }
  }

  // --------------------------------------------------------
  // (2) Read FSR and calculate voltage
  // ---------------------------------------------------------
  fsrValue = analogRead(fsrSensorPin);
  U_adc    = (fsrValue / 1023.0) * 5000.0;

  // ---------------------------------------------------------
  // (3) Floor 2 & 3 -> automatic disable after 2s
  // ---------------------------------------------------------
  bool floor2Blocked = (floorValues[1] > 100);
  if (floor2Blocked) {
    if (floor2BlockedSince == 0) {
      floor2BlockedSince = currentMicros;
    } else {
      if (!floor2Disabled && (currentMicros - floor2BlockedSince >= 2000000)) {
        floor2Disabled = true;
        Serial.println("Floor sensor 2 (Index 1) disabled (blocked >2s).");
      }
    }
  } else {
    floor2BlockedSince = 0;
    floor2Disabled     = false;
  }

  bool floor3Blocked = (floorValues[2] > 100);
  if (floor3Blocked) {
    if (floor3BlockedSince == 0) {
      floor3BlockedSince = currentMicros;
    } else {
      if (!floor3Disabled && (currentMicros - floor3BlockedSince >= 2000000)) {
        floor3Disabled = true;
        Serial.println("Floor sensor 3 (Index 2) disabled (blocked >2s).");
      }
    }
  } else {
    floor3BlockedSince = 0;
    floor3Disabled     = false;
  }

  // ---------------------------------------------------------
  // (4) Interruption logic (Floor & FSR)
  // ---------------------------------------------------------
  bool floorInterrupted =
      (floorValues[0] > 100)
      || ((floorValues[1] > 100) && !floor2Disabled)
      || ((floorValues[2] > 100) && !floor3Disabled);

  bool fsrInterrupted  = (U_adc < 1000.0);

  // ---------------------------------------------------------
  // (5) FSR control LEDs
  //     fsrInterrupted = TRUE  => LED OFF
  //                      FALSE => LED ON
  // ---------------------------------------------------------
  for (int i = 0; i < 3; i++) {
    digitalWrite(fsrControlLEDs[i], fsrInterrupted ? LOW : HIGH);
  }

  // ---------------------------------------------------------
  // (6) Edge detection in IDLE
  // ---------------------------------------------------------
  if (currentState == IDLE) {
    // Floor edge
    if (floorInterrupted && !floorInterruptedOld) {
      currentState     = WAITING_FOR_FSR;
      firstMicrosFloor = currentMicros;
      startWaitMicros  = currentMicros;
      Serial.println("IDLE -> WAITING_FOR_FSR (Floor detected first)");
    }
    // FSR edge
    else if (fsrInterrupted && !fsrInterruptedOld) {
      currentState    = WAITING_FOR_FLOOR;
      firstMicrosFSR  = currentMicros;
      startWaitMicros = currentMicros;
      Serial.println("IDLE -> WAITING_FOR_FLOOR (FSR detected first)");
    }
  }

  // ---------------------------------------------------------
  // (7) State machine
  // ---------------------------------------------------------
  switch (currentState) {
    case IDLE:
      // Waiting for edges
      break;

    case WAITING_FOR_FSR:
      // Floor was first -> wait for FSR
      if ((currentMicros - startWaitMicros) > resetWindow) {
        Serial.println("WAITING_FOR_FSR aborted -> IDLE");
        currentState = IDLE;
      }
      else if (fsrInterrupted) {
        if ((currentMicros - firstMicrosFloor) <= triggerWindow) {
          // "IN"
          triggerGreen(currentMicros); 
        } else {
          Serial.println("WAITING_FOR_FSR: triggerWindow expired -> IDLE");
        }
        currentState = IDLE;
      }
      break;

    case WAITING_FOR_FLOOR:
      // FSR was first -> wait for Floor
      if ((currentMicros - startWaitMicros) > resetWindow) {
        Serial.println("WAITING_FOR_FLOOR aborted -> IDLE");
        currentState = IDLE;
      }
      else if (floorInterrupted) {
        if ((currentMicros - firstMicrosFSR) <= triggerWindow) {
          // "OUT"
          triggerRed(); 
        } else {
          Serial.println("WAITING_FOR_FLOOR: triggerWindow expired -> IDLE");
        }
        currentState = IDLE;
      }
      break;
  }

  // ---------------------------------------------------------
  // (8) LED timeouts
  // ---------------------------------------------------------
  if (isGreenActive && (currentMicros - firstMicrosFloor >= interval)) {
    digitalWrite(greenLED_ext, LOW);
    digitalWrite(greenLED_int, LOW);
    isGreenActive = false;
    Serial.println("Green LED auto-off after 2s.");
  }
  if (isRedActive && (currentMicros - firstMicrosFSR >= interval)) {
    digitalWrite(redLED_ext, LOW);
    digitalWrite(redLED_int, LOW);
    isRedActive = false;
    Serial.println("Red LED auto-off after 2s.");
  }

  // ---------------------------------------------------------
  // (9) Debug output
  // ---------------------------------------------------------
  Serial.println(U_adc);

  // ---------------------------------------------------------
  // (10) Edge update
  // ---------------------------------------------------------
  floorInterruptedOld = floorInterrupted;
  fsrInterruptedOld   = fsrInterrupted;
}