#ifndef KalmanFilter_h
#define KalmanFilter_h

class KalmanFilter {
  public:
    KalmanFilter() {}

    float process(float z) {

      //constants
      static const float reading_rate = 2.55; // Time step (ms)
      static const float H = 1.0; //measurement map scalar
      static const float R = 20; //measurement noise covariance --> the higher the smoother
      static float Q = 0.01; //process noise covariance
      static float P = 1.0; //initial state covariance

      //state variable
      static float x = 0.0; //estimated state
      static float x_last = 0.0; //previous state
      static float K = 0.0; //kalman gain
      static float S = 0.0; //innovation covariance
      static float gradient_change_rate = 0.0; //rate of change of state

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
};

#endif
