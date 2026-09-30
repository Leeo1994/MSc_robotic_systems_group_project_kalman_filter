#include "LineSensors.h"
#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "PID.h"
Motors_c motors;
LineSensors_c line_sensors;
Kinematics_c  pose;


PID_c theta_pid;
PID_c left_pid;
PID_c right_pid;

#define BUTTON_A 14
#define BUTTON_B 30
#define BUTTON_C 17

int RAW = 0;
int EMA  = 1;
int KF = 2;

#define START 0
#define CALIBRATE 1
#define END_CALIBRATE 2
#define SETUP_TRIAL 3
#define TRIAL 4
#define END_TRIAL 5
#define UPLOAD_RESULTS 6
#define END 7

#define PID_UPDATE_MS 20
#define SPEED_EST_MS 10

#define PID_LEFT_P 1.001
#define PID_LEFT_I 0.001
#define PID_LEFT_D 0.0
#define PID_RIGHT_P 1.001
#define PID_RIGHT_I 0.001
#define PID_RIGHT_D 0.0

#define THETA_P 20.0
#define THETA_I 0.001
#define THETA_D 0.001
int filterMode;
int state ;

unsigned long speed_est_ts;
long last_e0;
float speed_e0;
long last_e1;
float speed_e1;
float alpha = 0.1;
float smoothed_speed_e0;
float smoothed_speed_e1;
float demand = 25.0;
float measurement_left = 0.0;
float measurement_right = 0.0;
unsigned long pid_ts;
unsigned long demand_ts;
float current_theta = 0;
float target_theta = 0;

//glob variables for data storage

float start_pose[2];
float end_pose[2];
float end_reading;

// Variables for button debouncing
int buttonAState = HIGH;
int lastButtonAState = HIGH;
unsigned long lastDebounceTimeA = 0;

int buttonBState = HIGH;
int lastButtonBState = HIGH;
unsigned long lastDebounceTimeB = 0;

int buttonCState = HIGH;
int lastButtonCState = HIGH;
unsigned long lastDebounceTimeC = 0;

unsigned long debounceDelay = 50; // Debounce delay in milliseconds


typedef struct {
  uint8_t id;
  float pose_x;
  float pose_y;
  float pose_theta;
  float reading;
} Trial;

Trial trial1, trial2, trial3, trial4, trial5, trial6, trial7, trial8, trial9, trial10;


uint8_t current_trial = 1;

void setup() {
  Serial.begin(9600);
  motors.initialise();
  line_sensors.initialiseForADC();

  // Set up button pins with internal pull-up resistors
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);



  state = START;
  filterMode = RAW;
}

void loop() {
  // Check buttons and update state
  checkButton(BUTTON_A, buttonAState, lastButtonAState, lastDebounceTimeA, CALIBRATE);
  checkButton(BUTTON_B, buttonBState, lastButtonBState, lastDebounceTimeB, SETUP_TRIAL);
  checkButton(BUTTON_C, buttonCState, lastButtonCState, lastDebounceTimeC, UPLOAD_RESULTS);


  if (state == START) {

  }

  else if (state == CALIBRATE) {
    delay(500);
    motors.startRotating(40);
    line_sensors.calibrateADC(10000);
    motors.stopRotating();
    // Calculate calibrated ADC values
    line_sensors.calcCalibratedADC();
    state = END_CALIBRATE;
  } else if (state == END_CALIBRATE) {

  } else if (state == SETUP_TRIAL) {
    //delay(500);
    setupEncoder0();
    setupEncoder1();
    pose.initialise(0, 0, 0);
    Serial.println("Threshold reached?");
    Serial.println(line_sensors.EMAThresholdReached(2));
    start_pose[0] = pose.x;
    start_pose[1] = pose.y;
    start_pose[2] = pose.theta;
    setPID();
    demand = 25;
    demand_ts = millis();
    speed_est_ts = millis();

    //    line_sensors.resetCalibrated();
    //    Serial.print("Threshold: ");
    //    Serial.println(line_sensors.calibrated[2]);

    state = TRIAL;

  } else if (state == TRIAL) {
    checkPID();
    //    Serial.println("State = TRIAL");
    //    Serial.print("filterMode:");
    //    Serial.println(filterMode);
    //    Serial.println();




    if (filterMode == RAW) {
      if ((line_sensors.rawThresholdReached(2)) == true) {
        motors.setPWM(0, 0);
        demand = 0;
        state = END_TRIAL;
        //        trial = pose.x;
        //        end_pose[1] = pose.y;
        //        end_pose[2] = pose.theta;
        //        end_reading = line_sensors.reading[1];
        assignTrial(pose.x, pose.y, pose.theta, line_sensors.reading[0]);
        current_trial++;
      }
    } else if (filterMode == EMA) {
      if ((line_sensors.EMAThresholdReached(2)) == true) {
        motors.setPWM(0, 0);
        demand = 0;
        state = END_TRIAL;
        //        end_pose[0] = pose.x;
        //        end_pose[1] = pose.y;
        //        end_pose[2] = pose.theta;
        //        end_reading = line_sensors.reading[1];
        assignTrial(pose.x, pose.y, pose.theta, line_sensors.reading[1]);
        current_trial++;

      }
    } else if (filterMode == KF) {
      if ((line_sensors.KFThresholdReached(2)) == true) {
        motors.setPWM(0, 0);
        demand = 0;
        state = END_TRIAL;
        //        end_pose[0] = pose.x;
        //        end_pose[1] = pose.y;
        //        end_pose[2] = pose.theta;
        //        end_reading = line_sensors.reading[2];
        assignTrial(pose.x, pose.y, pose.theta, line_sensors.reading[2]);
        current_trial++;

      }
    }
  } else if (state == END_TRIAL) {

  } else if (state == UPLOAD_RESULTS) {
    printTrial();
    //    Serial.print("\n----RESULTS----\n");
    //    Serial.print("\nStart Pose:");
    //    Serial.print(start_pose[0]);
    //    Serial.print(",");
    //    Serial.print(start_pose[1]);
    //    Serial.print(",");
    //    Serial.print(start_pose[2], 10);
    //    Serial.print("\n\nEnd Pose:");
    //    Serial.print(end_pose[0]);
    //    Serial.print(",");
    //    Serial.print(end_pose[1]);
    //    Serial.print(",");
    //    Serial.print(end_pose[2], 6);
    // Print based on the current filter mode
    //
    //    Serial.print("\n\nLast Reading:");
    //    Serial.print(line_sensors.reading[2], 4);

    state = END;

  } else if (state = END) {

  }




}

void checkButton(int buttonPin, int &buttonState, int &lastButtonState,
                 unsigned long &lastDebounceTime, int newState) {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis(); // Reset debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        state = newState;
      }
    }
  }

  lastButtonState = reading;
}
void setPID() {
  last_e0 = count_e0;
  speed_e0 = 0.0;
  smoothed_speed_e0 = 0.0;

  last_e1 = count_e1;
  speed_e1 = 0.0;
  smoothed_speed_e1 = 0.0;


  left_pid.initialise(PID_LEFT_P, PID_LEFT_I, PID_LEFT_D);
  right_pid.initialise(PID_RIGHT_P, PID_RIGHT_I, PID_RIGHT_D);
  theta_pid.initialise(THETA_P, THETA_I, THETA_D);

  speed_est_ts = millis();
  pid_ts = millis();


  left_pid.reset();
  right_pid.reset();
  theta_pid.reset();
}

void checkPID() {
  unsigned long elapsed_time = millis() - speed_est_ts;
  if ( elapsed_time >= SPEED_EST_MS) {
    long count_difference_e0 = count_e0 - last_e0;
    long count_difference_e1 = count_e1 - last_e1;

    last_e0 = count_e0;
    last_e1 = count_e1;

    speed_e0 = (float)count_difference_e0 / ((float)elapsed_time);
    speed_e1 = (float)count_difference_e1 / ((float)elapsed_time);

    smoothed_speed_e0 = (alpha * speed_e0) + ((1.0 - alpha) * smoothed_speed_e0);
    smoothed_speed_e1 = (alpha * speed_e1) + ((1.0 - alpha) * smoothed_speed_e1);

    measurement_left = smoothed_speed_e0;
    measurement_right = smoothed_speed_e1;

    speed_est_ts = millis();
    pose.update();

    //    Serial.print("Speed E0: "); Serial.print(speed_e0);
    //    Serial.print(", Speed E1: "); Serial.print(speed_e1);
    //    Serial.print(", Current Theta: "); Serial.print(current_theta);
    //    Serial.println();
  }

  if (millis() - pid_ts > PID_UPDATE_MS) {
    pose.update();
    current_theta = pose.theta;

    float theta_error = target_theta - current_theta;

    float theta_correction = theta_pid.update(target_theta, theta_error);

    float l_pwm = left_pid.update(demand + theta_correction, measurement_left);
    float r_pwm = right_pid.update(demand - theta_correction, measurement_right);
    motors.setPWM(l_pwm, r_pwm);

    pid_ts = millis();

    //    Serial.print(demand);
    //    Serial.print(",");
    //    Serial.print(l_pwm);
    //    Serial.print(",");
    //    Serial.print(r_pwm);
    //    Serial.print(",");
    //    Serial.print(theta_error);
    //    Serial.print(",");
    //    Serial.print(target_theta);
    //    Serial.print("\n");

    //
  }
}

void assignTrial(float pose_x, float pose_y, float pose_theta, float reading) {
  if (current_trial == 1) {
    trial1.id = 1;
    trial1.pose_x = pose_x;
    trial1.pose_y = pose_y;
    trial1.pose_theta = pose_theta;
    trial1.reading = reading;
  } else if (current_trial == 2) {
    trial2.id = 2;
    trial2.pose_x = pose_x;
    trial2.pose_y = pose_y;
    trial2.pose_theta = pose_theta;
    trial2.reading = reading;
  } else if (current_trial == 3) {
    trial3.id = 3;
    trial3.pose_x = pose_x;
    trial3.pose_y = pose_y;
    trial3.pose_theta = pose_theta;
    trial3.reading = reading;
  } else if (current_trial == 4) {
    trial4.id = 4;
    trial4.pose_x = pose_x;
    trial4.pose_y = pose_y;
    trial4.pose_theta = pose_theta;
    trial4.reading = reading;
  } else if (current_trial == 5) {
    trial5.id = 5;
    trial5.pose_x = pose_x;
    trial5.pose_y = pose_y;
    trial5.pose_theta = pose_theta;
    trial5.reading = reading;
  } else if (current_trial == 6) {
    trial6.id = 6;
    trial6.pose_x = pose_x;
    trial6.pose_y = pose_y;
    trial6.pose_theta = pose_theta;
    trial6.reading = reading;
  } else if (current_trial == 7) {
    trial7.id = 7;
    trial7.pose_x = pose_x;
    trial7.pose_y = pose_y;
    trial7.pose_theta = pose_theta;
    trial7.reading = reading;
  } else if (current_trial == 8) {
    trial8.id = 8;
    trial8.pose_x = pose_x;
    trial8.pose_y = pose_y;
    trial8.pose_theta = pose_theta;
    trial8.reading = reading;
  } else if (current_trial == 9) {
    trial9.id = 9;
    trial9.pose_x = pose_x;
    trial9.pose_y = pose_y;
    trial9.pose_theta = pose_theta;
    trial9.reading = reading;
  } else if (current_trial == 10) {
    trial10.id = 10;
    trial10.pose_x = pose_x;
    trial10.pose_y = pose_y;
    trial10.pose_theta = pose_theta;
    trial10.reading = reading;
  }

}

void printTrial() {
  Serial.print("\n----RESULTS 1----\n");
  Serial.print("\n----55%, 0.11298----\n");
  Serial.print(trial1.pose_x);
  Serial.print(",");
  Serial.print(trial1.pose_y);
  Serial.print(",");
  Serial.print(trial1.pose_theta);
  Serial.print(",");
  Serial.print(trial1.reading, 6);
  Serial.print("\n----END OF TRIAL 1----\n");

  Serial.print("\n----RESULTS 2----\n");
  Serial.print(trial2.pose_x);
  Serial.print(",");
  Serial.print(trial2.pose_y);
  Serial.print(",");
  Serial.print(trial2.pose_theta);
  Serial.print(",");
  Serial.print(trial2.reading, 6);
  Serial.print("\n----END OF TRIAL 2----\n");

  Serial.print("\n----RESULTS 3----\n");
  Serial.print(trial3.pose_x);
  Serial.print(",");
  Serial.print(trial3.pose_y);
  Serial.print(",");
  Serial.print(trial3.pose_theta);
  Serial.print(",");
  Serial.print(trial3.reading, 6);
  Serial.print("\n----END OF TRIAL 3----\n");

  Serial.print("\n----RESULTS 4----\n");
  Serial.print(trial4.pose_x);
  Serial.print(",");
  Serial.print(trial4.pose_y);
  Serial.print(",");
  Serial.print(trial4.pose_theta);
  Serial.print(",");
  Serial.print(trial4.reading, 6);
  Serial.print("\n----END OF TRIAL 4----\n");

  Serial.print("\n----RESULTS 5----\n");
  Serial.print(trial5.pose_x);
  Serial.print(",");
  Serial.print(trial5.pose_y);
  Serial.print(",");
  Serial.print(trial5.pose_theta);
  Serial.print(",");
  Serial.print(trial5.reading, 6);
  Serial.print("\n----END OF TRIAL 5----\n");

  Serial.print("\n----RESULTS 6----\n");
  Serial.print(trial6.pose_x);
  Serial.print(",");
  Serial.print(trial6.pose_y);
  Serial.print(",");
  Serial.print(trial6.pose_theta);
  Serial.print(",");
  Serial.print(trial6.reading, 6);
  Serial.print("\n----END OF TRIAL 6----\n");

  Serial.print("\n----RESULTS 7----\n");
  Serial.print(trial7.pose_x);
  Serial.print(",");
  Serial.print(trial7.pose_y);
  Serial.print(",");
  Serial.print(trial7.pose_theta);
  Serial.print(",");
  Serial.print(trial7.reading, 6);
  Serial.print("\n----END OF TRIAL 7----\n");

  Serial.print("\n----RESULTS 8----\n");
  Serial.print(trial8.pose_x);
  Serial.print(",");
  Serial.print(trial8.pose_y);
  Serial.print(",");
  Serial.print(trial8.pose_theta);
  Serial.print(",");
  Serial.print(trial8.reading, 6);
  Serial.print("\n----END OF TRIAL 8----\n");

  Serial.print("\n----RESULTS 9----\n");
  Serial.print(trial9.pose_x);
  Serial.print(",");
  Serial.print(trial9.pose_y);
  Serial.print(",");
  Serial.print(trial9.pose_theta);
  Serial.print(",");
  Serial.print(trial9.reading, 6);
  Serial.print("\n----END OF TRIAL 9----\n");

  Serial.print("\n----RESULTS 10----\n");
  Serial.print(trial10.pose_x);
  Serial.print(",");
  Serial.print(trial10.pose_y);
  Serial.print(",");
  Serial.print(trial10.pose_theta);
  Serial.print(",");
  Serial.print(trial10.reading, 6);
  Serial.print("\n----END OF TRIAL 10----\n");
}
