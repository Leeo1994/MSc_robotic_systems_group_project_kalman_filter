#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#include "ExponentialMovingAverageFilter.h"
#include "MovingAverageFilter.h"
#include "KalmanFilter.h"

#define NUM_SENSORS 5
#define EMIT_PIN 11

#define STOP_THRESHOLD 0.11298

const int sensor_pins[NUM_SENSORS] = {A11, A0, A2, A3, A4};

MovingAverageFilter filter(10); // Create a filter with a window size of 10
ExponentialMovingAverageFilter emaFilter(0.1); // Create a filter with alpha = 0.1
KalmanFilter kalmanFilter;

class LineSensors_c {

  public:
    float readings[NUM_SENSORS];
    float minimum[NUM_SENSORS];
    float maximum[NUM_SENSORS];
    float scaling[NUM_SENSORS];
    float calibrated[NUM_SENSORS];
    float range[NUM_SENSORS];
    float reading[2];

    unsigned long calibration_ts = 0;
    unsigned long calibration_stop_time = 0;
    bool is_calibrating = false;

    LineSensors_c() {}

    void initialiseForADC() {
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);

      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        pinMode(sensor_pins[sensor], INPUT_PULLUP);
        maximum[sensor] = 0;
        minimum[sensor] = 1023;
      }
    }

    void readSensorsADC() {
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        readings[sensor] = analogRead(sensor_pins[sensor]);
      }
    }

    void calibrateADC(unsigned long duration_ms) {
      calibration_ts = millis();
      calibration_stop_time = calibration_ts + duration_ms;
      is_calibrating = true;

      while (is_calibrating) {
        readSensorsADC();

        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
          if (readings[sensor] > maximum[sensor]) {
            maximum[sensor] = readings[sensor];
          }

          if (readings[sensor] < minimum[sensor]) {
            minimum[sensor] = readings[sensor];
          }
        }

        delay(10);
        checkCalibration();
      }

      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        range[sensor] = maximum[sensor] - minimum[sensor];
      }
    }

    void checkCalibration() {
      if (is_calibrating) {
        unsigned long currentMillis = millis();
        if (currentMillis > calibration_stop_time) {
          is_calibrating = false;
        }
      }
    }

    void calcCalibratedADC() {
      readSensorsADC();

      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        calibrated[sensor] = (readings[sensor] - minimum[sensor]) / range[sensor];
      }
      reading[0] = calibrated[2];
      reading[1] = emaFilter.process(calibrated[2]);
      reading[2] = kalmanFilter.process(calibrated[2]);
    }

    void resetCalibrated() {
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        calibrated[sensor] = 0;
      }
    }

    void print() {
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(calibrated[i] * 10, 4);
        if (i < NUM_SENSORS - 1) {
          Serial.print(",");
        } else {
          Serial.print("\n");
        }
      }
    }

    void printMAFiltered() {
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(filter.process(calibrated[i]) * 10, 4);
        if (i < NUM_SENSORS - 1) {
          Serial.print(",");
        } else {
          Serial.print("\n");
        }
      }
    }

    void printEMAFiltered() {
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(emaFilter.process(calibrated[i]) * 10, 4);
        if (i < NUM_SENSORS - 1) {
          Serial.print(",");
        } else {
          Serial.print("\n");
        }
      }
    }
    bool rawThresholdReached(int sensor) {
      calcCalibratedADC();
      if (calibrated[sensor] > STOP_THRESHOLD) {
        return true;
      } else {
        return false;
      }
    }
    bool KFThresholdReached(int sensor) {
      calcCalibratedADC();
      if (kalmanFilter.process(calibrated[sensor]) > STOP_THRESHOLD) {
        return true;
      } else {
        return false;
      }
    }
    bool EMAThresholdReached(int sensor) {
      calcCalibratedADC();
      if (emaFilter.process(calibrated[sensor]) > STOP_THRESHOLD) {
        return true;
      } else {
        return false;
      }
    }
};

#endif
