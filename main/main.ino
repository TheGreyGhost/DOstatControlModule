/*
 * Pin assignments:
 * D2 out = Red LED
 * D3 out = Yellow LED
 * D4 in = On/Off/Auto switch left position
 * D5 in = On/Off/Auto switch right position
 * D6 in = Toggle Switch
 * D7 out = Green LED
 * D9 out = Pump On/Off
 * D10 in = push button
 * 
 * A5 in = left trimpot
 * A4 in = right trimpot
 * A2 in = Controller Signal (voltage divided by 11)
 * 
 * Controls are:
 * 1) Red LED = flashes at a speed related to the percent of the threshold.  Slow flash = small percent.  Fast flash = large percent of threshold.  Solid = exceeded threshold
 * 2) Yellow LED = ON when the system is in the 10 minute post-cooldown mode
 * 3) Green LED = ON when the pump is running
 * 4) Left trimpot = trigger sensitivity
 * 5) Right trimpot = Pump run duration, dial is in minutes
 * 6) On/Off/Auto switch = pump mode selector
 * 7) Pushbutton = trigger dose immediately
 * 8) Toggle = Abort cooldown
 */

const bool TEST_MODE = true;

const int ANALOG_READ_10V = 865;  // the ANALOG_READ value on pin A2 when the controller signal is 10V
const int TRIMPOT_SENSITIVITY_MAX_VALUE = 20;  // the controller value (%) corresponding to the maximum value of the trimpot (1024 analog read, 10.0 on the dial)
const int CONTROLLER_MAX_VALUE = 100; // maximum controller value

void setup() {
  // put your setup code here, to run once:
  //start serial connection
  Serial.begin(9600);
  //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);
}

unsigned long lastSecondMillis;
unsigned long triggerMillis; // time that the DO-state was triggered
bool hasTriggeredPump = false; // set to true when the DO-stat has triggered
bool hasTriggeredCooldown = false; // set to true when the DO-stat has triggered

float redLEDflashHz; // rate at which to flash the red LED
unsigned long flashChangeMillis;  // time at which the last RED LED change in state occurred
bool redLEDon;  // what state is the redLED?

void loop() {
  bool buttonTriggerDose = (digitalRead(10) == LOW ? true : false);
  bool buttonAbortCooldown = (digitalRead(6) == LOW ? true : false);
  bool buttonRunPump = (digitalRead(4) == LOW ? true : false);
  bool buttonAutoMode = (digitalRead(5) == LOW ? true : false);

  int trimpotSensitivity = readTrimpot(A5);
  int trimpotRunDuration = readTrimpot(A4);
  int controllerSignal = readControllerSignal();

  unsigned long timeNowMillis = millis();

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
  double controllerDeviationThreshold = (double)TRIMPOT_SENSITIVITY_MAX_VALUE * (double)trimpotSensitivity / 1024;
  double controllerDeviation = getFilteredControllerDeviation(timeNowMillis, controllerSignal);
  redLEDflashHz = SLOWEST_FLASH_SPEED + (FASTEST_FLASH_SPEED - SLOWEST_FLASH_SPEED) * (controllerDeviation / controllerDeviationThreshold);
//  Serial.print(Deviation); Serial.print(" "); Serial.print(controllerSignal); Serial.print(" "); Serial.println(controllerDeviationThreshold);
  if (buttonAutoMode) {
    if (buttonTriggerDose || controllerDeviation >= controllerDeviationThreshold) {
      if (!hasTriggeredCooldown) {  
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
  digitalWrite(3, hasTriggeredCooldown ? HIGH : LOW); // yellow LED

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
  digitalWrite(2, redLEDon ? HIGH : LOW); // red LED
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

const int STORE_BUFFER_SIZE = 60 * 5;  // 5 minutes
int bufferpos = 0;  // the position of the next location to store a controull
int buffer[STORE_BUFFER_SIZE];  // buffer to store the controller output values for the recent past
double bufferAverage;  // the average value of the buffer

const unsigned long SAMPLE_PERIOD_MILLIS = 1000;
unsigned long lastDatapointMillis; // time that the last datapoint was written
bool firstSample = true;
int sampleCount = 0;
long sampleSum = 0;
const int GLITCH_BUFFER_SIZE = 10;
int glitchRejectBuffer[GLITCH_BUFFER_SIZE];

double getFilteredControllerDeviation(unsigned long timeNowMillis, int currentReading) {
  if (firstSample) {
    lastDatapointMillis = timeNowMillis;
    sampleCount = 0;
    sampleSum = 0;
    firstSample = false;
  }
  if (timeNowMillis - lastDatapointMillis < SAMPLE_PERIOD_MILLIS) {
    if (sampleCount < GLITCH_BUFFER_SIZE) glitchRejectBuffer[sampleCount] = currentReading;
    ++sampleCount;
    sampleSum += currentReading;
  } else {
    unsigned long overshoot = (timeNowMillis - lastDatapointMillis) % SAMPLE_PERIOD_MILLIS;
    lastDatapointMillis = timeNowMillis - overshoot;
    double meanValue = (sampleCount > 0 ? (double)sampleSum / sampleCount : 0);
    int minval = 10000;
    int maxval = -1;
    for (int i=0; i< sampleCount; ++i) {
      Serial.print(spacecheck[i]); Serial.print(" "); 
      minval = (minval < spacecheck[i] ? minval : spacecheck[i]);
      maxval = (maxval > spacecheck[i] ? maxval : spacecheck[i]);
    }
    
    Serial.print(sampleCount); Serial.print(" "); Serial.print(meanValue); Serial.print(" "); Serial.print(minval); Serial.print(" "); Serial.println(maxval);
    sampleCount = 0;
    sampleSum = 0;    
  }
  return (double)CONTROLLER_MAX_VALUE * (double)currentReading / (double)ANALOG_READ_10V;
}
