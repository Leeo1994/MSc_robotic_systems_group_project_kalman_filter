#ifndef ExponentialMovingAverageFilter_h
#define ExponentialMovingAverageFilter_h

class ExponentialMovingAverageFilter {
public:
  ExponentialMovingAverageFilter(float alpha) {
    this->alpha = constrain(alpha, 0.0, 1.0); // Ensure alpha is between 0 and 1
    initialized = false;
    ema = 0.0;
  }

  float process(float input) {
    if (!initialized) {
      ema = input; // Initialize EMA with the first input
      initialized = true;
    } else {
      ema = alpha * input + (1 - alpha) * ema; // EMA formula
    }
    return ema;
  }

  void reset() {
    initialized = false;
    ema = 0.0;
  }

  float getEMA() const { return ema; }

private:
  float alpha;      // Smoothing factor between 0 and 1
  bool initialized; // Flag to check if EMA has been initialized
  float ema;        // Current EMA value
};

#endif
