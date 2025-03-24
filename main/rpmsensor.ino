// measure the rpm of the fermenter as detected by an optical sensor
// hardware setup: open collector signal applied to pin 2 (D2)

// used to measure the speed of the fermenter as detected by pulses
// An Interrupt is used to update the values whenever the leading edge of a pulse is detected
// usage:
// 1) call rpmSensorSetup() once during setup
// 2) when an rpm readie is desired, call readRPM()
// initialise previousTriggerTime and mostrecentTriggerTime to zero

volatile byte criticalSectionCounter;
volatile unsigned long previousTriggerTime;
volatile unsigned long mostrecentTriggerTime;

unsigned long lastPrintoutTimeMillis = 0;
const unsigned long PRINTOUT_INTERVAL_MILLIS = 60000;

void rpmSensorSetup() {
  pinMode(2, INPUT_PULLUP);
  previousTriggerTime = 0;
  mostrecentTriggerTime = 0;
  
  attachInterrupt(digitalPinToInterrupt(2), onDetectInterrupt, FALLING);
  histogramClear();
  lastPrintoutTimeMillis = millis();
}

const bool ECHO_EVERY_RPM_MEASUREMENT = false;

const int HISTOGRAM_TABLE_SMALLEST_ENTRY = (RPM_MIN_VALID/2);
const int HISTOGRAM_TABLE_ENTRIES_COUNT = (RPM_MAX_VALID/2) - (RPM_MIN_VALID/2) + 1;
const unsigned short HISTOGRAM_MAX_COUNT = 30000;
unsigned short histogram[HISTOGRAM_TABLE_ENTRIES_COUNT];  // each table entry covers two rpm eg 360 and 361 map to the same cell
unsigned short histogramInvalidCount;
unsigned short histogramSampleCount;

void testRpmSensorloop() {
  static byte lastCriticalSectionCounter;
  byte rpmcounter = criticalSectionCounter;

  if (rpmcounter == lastCriticalSectionCounter) return; // no updated value yet
  lastCriticalSectionCounter = rpmcounter;
  
  float rpm = readRPM();
  if (ECHO_EVERY_RPM_MEASUREMENT) Serial.println(rpm);
  bool isFull = addValueToHistogram(rpm);
  if (isFull || (millis() - lastPrintoutTimeMillis > PRINTOUT_INTERVAL_MILLIS)) {
    printHistogram();
    histogramClear();
    lastPrintoutTimeMillis = millis();
  }
}

// returns true if histogram is full
bool addValueToHistogram(float value) {
  ++histogramSampleCount;
  if (value < 0) {
    if (histogramInvalidCount >= HISTOGRAM_MAX_COUNT) return true;
    ++ histogramInvalidCount;
    return false;
  }

  int rpmInt = (int)value;
  if (rpmInt < RPM_MIN_VALID) rpmInt = RPM_MIN_VALID;
  if (rpmInt > RPM_MAX_VALID) rpmInt = RPM_MAX_VALID;
  rpmInt -= RPM_MIN_VALID;
  rpmInt /= 2;
  if (rpmInt >= HISTOGRAM_TABLE_ENTRIES_COUNT) rpmInt = HISTOGRAM_TABLE_ENTRIES_COUNT - 1;
  if (histogram[rpmInt] >= HISTOGRAM_MAX_COUNT) return true;
  ++histogram[rpmInt];
  return false;
}

void histogramClear() {
  for (int i = 0; i < HISTOGRAM_TABLE_ENTRIES_COUNT; ++i) histogram[i] = 0;
  histogramInvalidCount = 0;
  histogramSampleCount = 0;
}

void printHistogram() {
  Serial.print(F("Total samples:"));
  Serial.println(histogramSampleCount);

  Serial.print(F("Invalid samples:"));
  Serial.println(histogramInvalidCount);
  int column = 0;
  int i = 0;
  do {
    if (column == 0) {
      Serial.print(i*2 + RPM_MIN_VALID);
      Serial.print(":");
    }
    Serial.print(histogram[i]);
    Serial.print(" ");
    if (++column == 5) {
      column = 0;
      Serial.println();
    }
  } while (++i < HISTOGRAM_TABLE_ENTRIES_COUNT);
}

void onDetectInterrupt()
{
  /* The Arduino calls this function when
     it detects a falling edge on pin 2. */
  previousTriggerTime = mostrecentTriggerTime;
  mostrecentTriggerTime = micros();
  ++criticalSectionCounter;
}

const unsigned long MICROSECONDS_PER_MINUTE = 1000000UL * 60;

const unsigned long RPM_MAX_ELAPSED_TIME_MICROSECONDS = MICROSECONDS_PER_MINUTE / RPM_MIN_VALID;
const unsigned long RPM_MIN_ELAPSED_TIME_MICROSECONDS = MICROSECONDS_PER_MINUTE / RPM_MAX_VALID;

// when the current speed reading is needed:
// 1) read and store the value of criticalSectionCounter
// 2) copy previousTriggerTime and mostrecentTriggerTime into new variables
// 3) read the value of criticalSectionCounter; if it has changed, previousTriggerTime and mostrecentTriggerTime are invalid; repeat from step (1)
// 4) check if the reading is stale (compare mostrecentTriggerTime to micros())

// get the current RPM reading, returns:
//    zero if no valid reading - too slow (the most recent signal from the sensor is more than RPM_TIMEOUT_MICROSECONDS microseconds ago)
//    negative if no valid reading - too fast (the elapsed time between triggers is infeasibly fast)
float readRPM() {
  byte cscStart;
  unsigned long previous;
  unsigned long current;
  const int MAX_RETRIES = 3;

// repeat until the interrupt didn't trigger during our measurement
  for (int i = 0; i < MAX_RETRIES; ++i) {  
    cscStart = criticalSectionCounter;
    previous = previousTriggerTime;
    current = mostrecentTriggerTime;
    if (cscStart == criticalSectionCounter) break;  
  } 
  if (micros() - current >= RPM_MAX_ELAPSED_TIME_MICROSECONDS) return 0.0; // value is stale
  unsigned long elapsedTime = current - previous;
  if (elapsedTime < RPM_MIN_ELAPSED_TIME_MICROSECONDS) return -1.0; // value is infeasibly fast
  if (elapsedTime > RPM_MAX_ELAPSED_TIME_MICROSECONDS) return 0.0; // value is too slow (below minimum)
  return ((float)MICROSECONDS_PER_MINUTE / elapsedTime);
}
