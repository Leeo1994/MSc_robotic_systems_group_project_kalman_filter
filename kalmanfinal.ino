#ifndef KalmanFilter_h
#define KalmanFilter_h
 
#include <stdint.h> //change 1: needed for uint8_t
 
class KalmanFilter {
  public: 
    KalmanFilter() : isFirstRun(true) {} //initialize isFirstRun
 
    float process(float z) {
      
      //constants
      static const float reading_rate = 2.55; //time step (ms)
      static const float H = 1.0; //measurement map scalar
      static const float R_MIN = 1.0; //change 2: floor for adaptive R (tune to your sensor)
      static const float R_MAX = 100.0; //change 2: ceiling for adaptive R (tune to your sensor)
      static float R = 20; //measurement noise covariance --> the higher the smoother
      static float Q = 0.01; //process noise covariance
      static float P = 1.0; //initial state covariance
      
      //state variable
      static float x = 0.0; //estimated state
      static float x_last = 0.0; //previous state
      static float K = 0.0; //kalman gain
      static float S = 0.0; //innovation covariance
      static float gradient_change_rate = 0.0; //rate of change of state
 
      //R update variables:
      static float z_last_5[5] = {0}; //change 3: now static so the last 5 readings persist between calls
      static uint8_t current_idx = 0;
 
      //store current reading
      z_last_5[current_idx] = z;
 
      //increment + wrap index
      current_idx = (current_idx + 1) % 5; 
 
      //first run
      if (isFirstRun) {
        x = z; //initialize with first sensor reading
        isFirstRun = false; //set to false after first reading. 
      }
 
 
//--------------BUILDING ERROR COVARIANCES--------------------
 
//IN SYSTEM:
 
//assume large steps occur every ___ timestamps. - 
 
//CHECK WHEN THE LARGE STEPS OCCUR AND BASE YOURSELF OFF OF THAT!
 
//ASK KORY FOR FULL GRADIENT RUN DATA. 
 
//ASK CHAT OR COPILOT -
//1. IF I CORRECT MY MODEL TO ANTICIPATE A LARGE INCREASE EVERY ___ TIMESTAMPS AND TO DELTA IT AND EXPECT THE SAME INCREASE EVERY SAME TIMESTAMP - HOW DO I DEFINE ERROR COVARIANCE (Q) IN THIS MODEL?
//2. CAN I INCORORATE BOTH ASSUMPTION OF DELTA INCREASE BASED ON LAST READING AND A LARGE INCREASE EVERY __ ?
 
//IN READING:
 
if (current_idx == 0){
float mean = 0;
float var = 0;
 
  
//calc mean:
for (uint8_t i = 0; i < 5; i++){ //change 4: loop uses its own counter i instead of current_idx
mean += z_last_5[i];
}
mean /= 5.0; //divide accumulated mean by 5 to get avg mean
 
//calc variance:
for (uint8_t i = 0; i < 5; i++){ //change 4: same fix as above
var += (z_last_5[i] - mean) * (z_last_5[i] - mean);
}
 
var /= 5.0; //divide accumulated variance by 5 to get avg variance
 
R = var; //change 5: was R += var (R grew forever), now R tracks the latest window variance
if (R < R_MIN) R = R_MIN; //change 6: clamp so R never hits 0 or blows up at large steps
if (R > R_MAX) R = R_MAX; //change 6
}
 
//--------------MODEL-BASED-PREDICTION--------------------
 
      //predict gradient change rate
      gradient_change_rate = (x - x_last) / reading_rate;
 
      //predict next state (gradient value)
      float x_predicted = x_last + (gradient_change_rate * reading_rate);
      
      //update process error covariance
      P = P + Q;
 
//------------MEASUREMENT-BASED-CORRECTION-----------------      
 
      //calculate innovation covariance
      S = P + R;
      
      //calculate Kalman gain
      K = P / S;
 
      //correct state estimate
      x = x_predicted + K * (z - H * x_predicted);
 
      //update measurement error covariance
      P = P * (1 - K * H);
 
      //store current state for next iteration
      x_last = x;
 
      //return updated state
      return x;
    }
    
private:
    bool isFirstRun; 
};
 
#endif
