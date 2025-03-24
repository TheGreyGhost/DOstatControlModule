/* RPM monitoring

 measure the rpm of the fermenter as detected by an optical sensor
 hardware setup: open collector signal applied to pin 2

 used to measure the speed of the fermenter as detected by pulses
 An Interrupt is used to update the values whenever the leading edge of a pulse is detected
 usage:
 initialise previousTriggerTime and mostrecentTriggerTime to zero
 when the current speed reading is needed:
 1) read and store the value of criticalSectionCounter
 2) copy previousTriggerTime and mostrecentTriggerTime into new variables
 3) read the value of criticalSectionCounter; if it has changed, previousTriggerTime and mostrecentTriggerTime are invalid; repeat from step (1)
 4) check if the reading is stale (compare mostrecentTriggerTime to micros())
 
Usage:
  (1) construct a DataStats for each stream of data
  (2) call addDatapoint() each time a new datapoint is available
  (3) call clear() to reset the statistics

  read the various stats using:
    getCount, getMin, getMax, getAverage, getMostRecent   
 
  Note - the average will become inaccurate for large numbers of datapoints due to precision loss
    (the accuracy is approx = 6 digits minus the number of digits in the datapoint count)   
  */
#ifndef RpmSensor_h   // if x.h hasn't been included yet...
#define RpmSensor_h   //   #define this so the compiler knows it has been included
//#include <Arduino.h>

// set up the rpm sensor and start taking measurements
void rpmSensorSetup();

// get the current RPM reading, returns:
//    zero if no valid reading (the most recent signal from the sensor is more than RPM_TIMEOUT_MICROSECONDS microseconds ago)
//    negative if no valid reading (the elapsed time between triggers is infeasibly fast)
float readRPM();

// test function for the rpm sensor.  Call in a loop to collect sensor readings and present as histogram.
void testRpmSensorloop();

const unsigned long RPM_MIN_VALID = 40;
const unsigned long RPM_MAX_VALID = 480;

#endif
