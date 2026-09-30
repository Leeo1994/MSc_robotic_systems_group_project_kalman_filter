#ifndef MovingAverageFilter_h
#define MovingAverageFilter_h

class MovingAverageFilter {
public:
  MovingAverageFilter(uint8_t windowSize) {
    this->windowSize = windowSize;
    buffer = new float[windowSize];
    reset();
  }

  ~MovingAverageFilter() { delete[] buffer; }

  float process(float input) {
    sum -= buffer[index];  // Subtract the oldest value from sum
    buffer[index] = input; // Overwrite the oldest value with new input
    sum += input;          // Add the new input to sum

    index = (index + 1) % windowSize; // Move to the next index

    if (count < windowSize) {
      count++; // Keep track of the number of inputs until buffer is full
    }

    return sum / count; // Return the average
  }

  

  void reset() {
    for (uint8_t i = 0; i < windowSize; i++) {
      buffer[i] = 0.0;
    }
    index = 0;
    count = 0;
    sum = 0.0;
  }

private:
  float *buffer;
  uint8_t windowSize;
  uint8_t index;
  uint8_t count;
  float sum;
};

#endif
