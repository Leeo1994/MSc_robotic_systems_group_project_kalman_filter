#ifndef KalmanFilter_h
#define KalmanFilter_h

class KalmanFilter {
  public: 
    KalmanFilter() {}

    float process(float z) {

    if (!initialized) {
      x = z;
      x_last = z;
      initialized = true;
      return x;
    }

     //constants
      static const float reading_rate = 2.55; //(milliseconds)
      static const float H = 1; //measurement map scalar (const for linear)
      static const float R = 0.1; //noise in measurement covariance
      static float Q = 0.01; //noise (disturbance) in system - initial estimate covariance

      //local variables
      //z = current measurement 
      float K = 0; //initial kalman gain
      float S = 0; //innovation covartiance - total uncertainty in the predicted state and sensor reading
      float x_predicted = 0; //initial x_predicted
      
//--------------MODEL-BASED-PREDICTION--------------------

//model itroduces gradient-change rate estimation:

//calc gradient rate of change
gradient_change_rate = (x - x_last) / reading_rate;

//predict next gradient value
x_predicted = x + (gradient_change_rate * reading_rate); //predicted gradient increase

//store current estimate for next round
x_last = x; //<---- was last_x

//update error covariance
P = P + Q;


//------------MEASUREMENT-BASED-CORRECTION-----------------

//update S - total uncertainty in predicted measurement
S = P + R; //where P is the predicted error covariance and R is the measurement noise covariance

//update kalman gain
//larger Kalman gain --> trust the sensor reading more 
//smaller Kalman gain --> trust the model prediction more
K = P / S; // (after update = P + Q / P + S)
//old formula --> K = (P * H) / (H * P * H + R);

//state correction - update estimate with current measurement 
x = x_predicted + K * (z - H * x_predicted); //(diff between measurement and predicted state - called innovation/residual.)

//update error covariance (reduce uncertainty based on kalman gain)
P = (1 - K * H) * P;

//return uopdated estimate of state x 
return x;

    }//end process float z

    float getGradient() const {return gradient_change_rate;}

  private: 
      //state variables
      bool initialized = false;
      float P = 1; //State covariance matrix - uncertainty in estimate x
      float x = 0; //initial state estimate
      float x_last = 0; //last x value - used for rate of change.
      float gradient_change_rate = 0; //initial gradient change rate
      
}; //end class

#endif
