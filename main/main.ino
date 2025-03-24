#include "rpmsensor.h"
/*
 * Pin assignments:
 * D2 in = rpm sensor pulse input
 * D3 out = Yellow LED
 * D4 in = On/Off/Auto switch left position
 * D5 in = On/Off/Auto switch right position
 * D6 in = Toggle Switch
 * D7 out = Green LED
 * D9 out = Pump On/Off
 * D10 in = push button
 * D11 out = Red LED
 * 
 * A5 in = left trimpot
 * A4 in = right trimpot
 * A2 in = Controller Signal (voltage divided by 11)
 * 
 * Controls are:
 * 1) Red LED = flashes at a speed related to the percent of the threshold.  Slow flash = small percent.  Fast flash = large percent of threshold.  Solid = exceeded threshold
 * 2) Yellow LED = ON when the system is in the 10 minute post-cooldown mode.  Flashes when the auto mode is not ready (long term average not collected yet)
 * 3) Green LED = ON when the pump is running
 * 4) Left trimpot = trigger sensitivity
 * 5) Right trimpot = Pump run duration, dial is in minutes
 * 6) On/Off/Auto switch = pump mode selector
 * 7) Pushbutton = trigger dose immediately
 * 8) Toggle = Abort cooldown
 */

const bool TEST_MODE = false;
const bool TEST_MODE_RPM = true;

const int ANALOG_READ_10V = 859;  // the ANALOG_READ value on pin A2 when the controller signal is 10V
const int TRIMPOT_SENSITIVITY_MAX_VALUE = 10;  // the controller value (%) corresponding to the maximum value of the trimpot (1024 analog read, 10.0 on the dial)
const int CONTROLLER_MAX_VALUE = 100; // maximum controller value

enum class InputSource {ANALOG, RPM, DONT_KNOW};
enum class RpmValidity {VALID, INVALID, DONT_KNOW};

void setup() {
  // put your setup code here, to run once:
  //start serial connection
  Serial.begin(9600);
  rpmSensorSetup();

  //configure pin 4 as an input and enable the internal pull-up resistor
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);

  pinMode(3, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);
}

unsigned long lastSecondMillis;
unsigned long triggerMillis; // time that the DO-state was triggered
bool hasTriggeredPump = false; // set to true when the DO-stat has triggered
bool hasTriggeredCooldown = false; // set to true when the DO-stat has triggered

float redLEDflashHz; // rate at which to flash the red LED
unsigned long flashChangeMillis;  // time at which the last RED LED change in state occurred
bool redLEDon;  // what state is the redLED?

unsigned long lastControllerPrintMillis;

void loop() {
  if (TEST_MODE_RPM) testRpmSensorloop();

  bool buttonTriggerDose = (digitalRead(10) == LOW ? true : false);
  bool buttonAbortCooldown = (digitalRead(6) == LOW ? true : false);
  bool buttonRunPump = (digitalRead(4) == LOW ? true : false);
  bool buttonAutoMode = (digitalRead(5) == LOW ? true : false);

  int trimpotSensitivity = readTrimpot(A5);
  int trimpotRunDuration = readTrimpot(A4);
  int controllerSignal = readControllerSignal();
  InputSource inputSource = whichInputSource(controllerSignal);
  if (inputSource == InputSource::RPM) {
    float rpm = readRPM();
    if (rpm >= RPM_MIN_VALID && rpm <= RPM_MAX_VALID) {  // rescale RPM to mimic the analog read value
      controllerSignal = static_cast<int>(ANALOG_READ_10V * rpm / (RPM_MAX_VALID - RPM_MIN_VALID));
    } else {
      inputSource = InputSource::DONT_KNOW;
    }
  } else if (inputSource == InputSource::DONT_KNOW) {
    resetControllerTrainingPeriod();
  }
  unsigned long timeNowMillis = millis();
  if (TEST_MODE && timeNowMillis - lastControllerPrintMillis > 1000) {
    lastControllerPrintMillis = timeNowMillis;
    Serial.print(F("controllerSignal:")); Serial.println(controllerSignal);
  }

  // check if cooldown has expired
  if (hasTriggeredCooldown) {
    unsigned long cooldownDurationMillis = 10UL*60UL*1000UL; // 10 minutes in milliseconds
    if (TEST_MODE) cooldownDurationMillis /= 30;
    if (buttonAbortCooldown || (timeNowMillis - triggerMillis >= cooldownDurationMillis)) {
      hasTriggeredCooldown = false;
    }
  }

  const float SLOWEST_FLASH_SPEED = 0.25; // Hz
  const float FASTEST_FLASH_SPEED = 10; // Hz
  const float NOT_READY_FLASH_SPEED = 0.5; //Hz
  double controllerDeviationThreshold = (double)TRIMPOT_SENSITIVITY_MAX_VALUE * (double)trimpotSensitivity / 1024;
  double controllerDeviation = 0;
  if (inputSource != InputSource::DONT_KNOW) {
    controllerDeviation = getFilteredControllerDeviation(timeNowMillis, controllerSignal);
  }
  bool autoControlReady = (controllerDeviation >=0);

  if (autoControlReady) {
    redLEDflashHz = SLOWEST_FLASH_SPEED + (FASTEST_FLASH_SPEED - SLOWEST_FLASH_SPEED) * (controllerDeviation / controllerDeviationThreshold);
  } else {
    redLEDflashHz = NOT_READY_FLASH_SPEED;
  }
//  Serial.print(Deviation); Serial.print(" "); Serial.print(controllerSignal); Serial.print(" "); Serial.println(controllerDeviationThreshold);
  if (buttonAutoMode) {
    if (buttonTriggerDose || (autoControlReady && inputSource != InputSource::DONT_KNOW && controllerDeviation >= controllerDeviationThreshold)) {
      if (!hasTriggeredCooldown) {  
        if (!hasTriggeredPump) Serial.println(F("Triggered"));
        hasTriggeredPump = true;
        hasTriggeredCooldown = (buttonAbortCooldown ? false : true);
        triggerMillis = timeNowMillis;
      }
    }
  }

  // Check if the pump should be running
  bool runPump;
  if (buttonRunPump) {
    runPump = true;
  } else if (!buttonAutoMode) {
    runPump = false;
  } else {
    if (hasTriggeredPump) {
      unsigned long pumpDurationMillis = ((unsigned long)trimpotRunDuration*10UL*60UL*1000UL)/1024;  // scale the analog maximum of 1024 to 10 minutes in milliseconds
      if (TEST_MODE) pumpDurationMillis /= 10;
      if (timeNowMillis - triggerMillis < pumpDurationMillis) {
        runPump = true;
      } else {
        runPump = false;
        hasTriggeredPump = false;
      }
    } else {
      runPump = false;  
    }
  }
  digitalWrite(9, runPump ? LOW : HIGH);  // output to pump
  digitalWrite(7, runPump ? HIGH : LOW);  // green LED

  if (redLEDflashHz >= FASTEST_FLASH_SPEED) {
    redLEDon = true;
  } else {
    unsigned long flashPeriodMillis = 1000 / redLEDflashHz;
    unsigned long flashHalfPeriodMillis = 1000 / redLEDflashHz / 2;
    if (timeNowMillis - flashChangeMillis >= flashHalfPeriodMillis) {  // change flash state
      unsigned long residualTime = (timeNowMillis - flashChangeMillis) % flashPeriodMillis;  // how far are we into the next flash?  Skip full flashes if any
      if (residualTime >= flashHalfPeriodMillis) {  // only change LED state if we didn't wrap all the way into the next flash
        redLEDon = !redLEDon;
      }
      residualTime %= flashHalfPeriodMillis;
      flashChangeMillis = timeNowMillis - residualTime;
    }
  }
  digitalWrite(11, redLEDon ? HIGH : LOW); // red LED
  if (autoControlReady) {
    digitalWrite(3, hasTriggeredCooldown ? HIGH : LOW); // yellow LED
  } else {
    digitalWrite(3, !redLEDon ? HIGH : LOW); // yellow LED    
  }
}

int readControllerSignal() {
  analogReference(INTERNAL);
  for (int repeats = 0; repeats < 100; ++repeats) {  // need at least 30 or so in order to account for the change in Reference voltage
    analogRead(A2);
  }
  return analogRead(A2);
}

int readTrimpot(int trimpot) {
  analogReference(DEFAULT);
  for (int repeats = 0; repeats < 10; ++repeats) {  // 10 repeats appears to be plenty when switching from INTERNAL back to DEFAULT
    analogRead(trimpot);
  }
  return analogRead(trimpot);
}


void copystate(int from, int to) {
  int sensorVal = digitalRead(from);
  if (sensorVal == HIGH) {
    digitalWrite(to, LOW);
  } else {
    digitalWrite(to, HIGH);
  }

}

// add the currentReading to the filtered sample
// calculate the deviation of the 30 second moving average from the 5 minute moving average  (-100 - 100 controller units)

const int BASELINE_BUFFER_SIZE = (TEST_MODE ? 60 : 60 * 5);  // 5 minutes
int baselineBufferPos = 0;  // the position of the next location to store a controller output
int baselineBufferCount = 0;  // how many positions in the buffer are used?
int baselineBuffer[BASELINE_BUFFER_SIZE];  // circular buffer to store the controller output values for the recent past
unsigned long baselineBufferSum;  // the total sum of values in the buffer

const int SHORT_BUFFER_SIZE = 10;  // 10 seconds
int shortBufferPos = 0;  // the position of the next location to store a controller output
int shortBufferCount = 0;  // how many positions in the buffer are used?
int shortBuffer[SHORT_BUFFER_SIZE];  // circular buffer to store the controller output values for the recent past
unsigned long shortBufferSum;  // the total sum of values in the buffer

const unsigned long SAMPLE_PERIOD_MILLIS = 1000;
unsigned long lastDatapointMillis; // time that the last datapoint was written
bool firstSample = true;
int sampleCount = 0;
long sampleSum = 0;
//const int GLITCH_BUFFER_SIZE = 10;
//int glitchRejectBuffer[GLITCH_BUFFER_SIZE];

const int DEGLITCH_THRESHOLD = 5; // based on typical noise

int samplePrevious;
int sampleBeingDeglitched;
int deglitchedSampleCount = 0;
unsigned long deglitchedSampleSum = 0;

double filteredControllerDeviationLastValue;
bool filteredControllerDeviationAvailable  = false;

void resetControllerTrainingPeriod() {
  firstSample = true;
}

//returns positive value for a deviation
// if the deviation is less than zero (i.e. the controller output is increasing, not decreasing) then it returns 0
// if no value is available yet, returns a negative value
double getFilteredControllerDeviation(unsigned long timeNowMillis, int currentReading) {
  if (firstSample) {
    lastDatapointMillis = timeNowMillis;
    sampleCount = 0;
    sampleSum = 0;
    deglitchedSampleCount = 0;
    deglitchedSampleSum = 0;
    baselineBufferPos = 0;
    baselineBufferCount = 0;
    baselineBufferSum = 0;
    shortBufferPos = 0;
    shortBufferCount = 0;
    shortBufferSum = 0;
    filteredControllerDeviationAvailable = false;
    firstSample = false;
  }
  
  if (timeNowMillis - lastDatapointMillis < SAMPLE_PERIOD_MILLIS) {
//    if (sampleCount < GLITCH_BUFFER_SIZE) glitchRejectBuffer[sampleCount] = currentReading;
    ++sampleCount;
    sampleSum += currentReading;
    if (sampleCount == 1) {
      samplePrevious = currentReading;
    } else if (sampleCount == 2) {
      sampleBeingDeglitched = currentReading;
    } else {
      bool glitchFound = false;
      if (sampleBeingDeglitched > samplePrevious + DEGLITCH_THRESHOLD && sampleBeingDeglitched > currentReading + DEGLITCH_THRESHOLD) glitchFound = true;
      if (sampleBeingDeglitched < samplePrevious - DEGLITCH_THRESHOLD && sampleBeingDeglitched < currentReading - DEGLITCH_THRESHOLD) glitchFound = true;
      if (!glitchFound) {
        ++deglitchedSampleCount;
        deglitchedSampleSum += sampleBeingDeglitched;
      }
      sampleBeingDeglitched = currentReading;
    }
  } else {
    unsigned long overshoot = (timeNowMillis - lastDatapointMillis) % SAMPLE_PERIOD_MILLIS;
    lastDatapointMillis = timeNowMillis - overshoot;
    int meanValueTimes16 = (sampleCount > 0 ? (16*sampleSum) / sampleCount : 0);
    int deglitchedMeanValueTimes16 = (deglitchedSampleCount > 0 ? (16*deglitchedSampleSum) / deglitchedSampleCount : 0);
//    int minval = 10000;
//    int maxval = -1;
//    for (int i=0; i< sampleCount; ++i) {
//      Serial.print(glitchRejectBuffer[i]); Serial.print(" "); 
//      minval = (minval < glitchRejectBuffer[i] ? minval : glitchRejectBuffer[i]);
//      maxval = (maxval > glitchRejectBuffer[i] ? maxval : glitchRejectBuffer[i]);
//    }
    
//    Serial.print(sampleCount); Serial.print(" "); Serial.print(meanValueTimes16); Serial.print(" "); 
//    Serial.print((deglitchedSampleCount == sampleCount - 2) ? "" : "*");
//    Serial.print(deglitchedSampleCount); Serial.print(" "); Serial.print(deglitchedMeanValueTimes16);
    sampleCount = 0;
    sampleSum = 0;
    deglitchedSampleCount = 0;    
    deglitchedSampleSum = 0;

    if (shortBufferCount >= SHORT_BUFFER_SIZE) {  // remove old value, insert new value
      shortBufferSum -= shortBuffer[shortBufferPos];
    } else {
      ++shortBufferCount;
    }
    shortBufferSum += deglitchedMeanValueTimes16;
    shortBuffer[shortBufferPos] = deglitchedMeanValueTimes16;
    if (++shortBufferPos >= SHORT_BUFFER_SIZE) shortBufferPos = 0;

    double shortTermAverage = (double)CONTROLLER_MAX_VALUE * (double)shortBufferSum / SHORT_BUFFER_SIZE / 16.0 / (double)ANALOG_READ_10V;
 //   Serial.print(" shortave:"); Serial.println(shortTermAverage);

    if (baselineBufferCount >= BASELINE_BUFFER_SIZE) {  // remove old value, insert new value
      baselineBufferSum -= baselineBuffer[baselineBufferPos];
    } else {
      ++baselineBufferCount;
    }
    baselineBufferSum += deglitchedMeanValueTimes16;
    baselineBuffer[baselineBufferPos] = deglitchedMeanValueTimes16;
    if (++baselineBufferPos >= BASELINE_BUFFER_SIZE) baselineBufferPos = 0;

    double baselineAverage = (double)CONTROLLER_MAX_VALUE * (double)baselineBufferSum / BASELINE_BUFFER_SIZE / 16.0 / (double)ANALOG_READ_10V;
    filteredControllerDeviationLastValue = baselineAverage - shortTermAverage;
    if (filteredControllerDeviationLastValue < 0) filteredControllerDeviationLastValue = 0;
    filteredControllerDeviationAvailable = (baselineBufferCount >= BASELINE_BUFFER_SIZE && shortBufferCount >= SHORT_BUFFER_SIZE);
//    Serial.print(" shortave:"); Serial.print(shortTermAverage); Serial.print(" baseline:"); Serial.print(baselineAverage);
//    Serial.print(" deviation:"); Serial.println(filteredControllerDeviationLastValue);
  }
  return (filteredControllerDeviationAvailable ? filteredControllerDeviationLastValue : -1);
}

/*
 * The two input sources are:
 * 1) When the plug from the 20L fermenter is plugged in, plug pin 10 = Digital Input 2 is open -> pulled high by intl pullup, and a voltage is applied to plug pins 4(+) & 17 (gnd) = Analog2 in
 * 2) When the plug from the rpm sensor is plugged in, plug pin 10 = Digital Input 2 is open or pulled low (pin10), and plug pin 4 is tied to gnd via 1K --> Analog2 in
 * 
 * The input source mode changes if:
 * 1) If currently is DONT_KNOW, and the rpm sensor is returning valid readings for at least 10 seconds: swap to RPM and trigger relearning phase
 * 2) If currently is DONT_KNOW, and the rpm sensor does not return a valid reading for at least 10 seconds, swap to ANALOG and trigger relearning phase
 * 3) If currently is ANALOG, and the analog reading is less than threshold, and rpm sensor is returning valid readings for at least 10 seconds: swap to RPM and trigger relearning phase
 * 4) If currently is RPM, and the rpm sensor does not return valid readings for at least 10 seconds, swap to ANALOG and trigger relearning phase
 */

InputSource inputSource = InputSource::DONT_KNOW;

const int THRESHOLD_CONTROLLER_SIGNAL = ANALOG_READ_10V / 20;
const float THRESHOLD_RPM_VALUE = RPM_MIN_VALID / 2;
const int INPUTSOURCE_CHANGEOVER_DELAY_MILLIS = 10 * 1000; //10 seconds
unsigned long lastInvalidRPMMillis = 0;   // time since last invalid rpm reading, clamped to INPUTSOURCE_CHANGEOVER_DELAY_MILLIS;
unsigned long lastValidRPMMillis = 0;   // time since last valid rpm reading, clamped to INPUTSOURCE_CHANGEOVER_DELAY_MILLIS;
unsigned long lastAnalogAboveThresholdMillis = 0;  // time since last reading below threshold, clamped to INPUTSOURCE_CHANGEOVER_DELAY_MILLIS;
bool firstRPMinvalid = true;
bool firstRPMvalid = true;

InputSource whichInputSource(int controllerSignal) {
    unsigned long currentMillis = millis();
    RpmValidity rpmValidity = RpmValidity::DONT_KNOW;
    float rpm = readRPM();
    if (rpm < THRESHOLD_RPM_VALUE) {
      lastInvalidRPMMillis = currentMillis;
      if (currentMillis - lastValidRPMMillis > INPUTSOURCE_CHANGEOVER_DELAY_MILLIS) {
        if (TEST_MODE_RPM && firstRPMinvalid) Serial.println(F("RPM invalid"));
        firstRPMinvalid = false;
        rpmValidity = RpmValidity::INVALID;
        lastValidRPMMillis = currentMillis - INPUTSOURCE_CHANGEOVER_DELAY_MILLIS;
      }
      firstRPMvalid = true;
    } else {
      lastValidRPMMillis = currentMillis;
      if (currentMillis - lastInvalidRPMMillis > INPUTSOURCE_CHANGEOVER_DELAY_MILLIS) {
        if (TEST_MODE_RPM && firstRPMvalid) Serial.println(F("RPM valid"));
        firstRPMvalid = false;
        rpmValidity = RpmValidity::VALID;
        lastInvalidRPMMillis = currentMillis - INPUTSOURCE_CHANGEOVER_DELAY_MILLIS;
      }
      firstRPMinvalid = true;
    }

    bool analogAboveThreshold = false;
    if (controllerSignal < THRESHOLD_CONTROLLER_SIGNAL) {
      lastAnalogAboveThresholdMillis = currentMillis;
    } else {
      if (currentMillis - lastAnalogAboveThresholdMillis > INPUTSOURCE_CHANGEOVER_DELAY_MILLIS) {
        analogAboveThreshold = true;
        lastAnalogAboveThresholdMillis = currentMillis - INPUTSOURCE_CHANGEOVER_DELAY_MILLIS;
      }
    }
  
    switch(inputSource) {
      case InputSource::DONT_KNOW: {
        if (rpmValidity == RpmValidity::VALID) {
          inputSource = InputSource::RPM;
          resetControllerTrainingPeriod();
        } else {
          if (rpmValidity == RpmValidity::INVALID) {
           inputSource = InputSource::ANALOG;
           resetControllerTrainingPeriod();           
          }
        }
        break;
      }
      case InputSource::ANALOG: {
         if (!analogAboveThreshold && rpmValidity == RpmValidity::VALID) {
          inputSource = InputSource::RPM;
          resetControllerTrainingPeriod();
         }
        break;
      }
      case InputSource::RPM: {
         if (rpmValidity == RpmValidity::INVALID) {
          inputSource = InputSource::ANALOG;
          resetControllerTrainingPeriod();
         }
        break;
      }
      default: return InputSource::DONT_KNOW;
    }
  return inputSource; 
}
