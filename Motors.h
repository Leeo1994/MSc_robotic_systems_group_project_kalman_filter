#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

#define FWD LOW
#define REV HIGH
#define MAX_PWM 60.0

class Motors_c {

public:
  bool is_rotating = false;

  Motors_c() {}

  void initialise() {
    pinMode(L_PWM, OUTPUT);
    pinMode(L_DIR, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(R_DIR, OUTPUT);
  }

  setPWM(float left_pwr, float right_pwr) {
    left_pwr = constrain(left_pwr, -MAX_PWM, MAX_PWM);
    right_pwr = constrain(right_pwr, -MAX_PWM, MAX_PWM);

    if (left_pwr < 0) {
      digitalWrite(L_DIR, REV);
      left_pwr = abs(left_pwr);
    } else {
      digitalWrite(L_DIR, FWD);
    }

    if (right_pwr < 0) {
      digitalWrite(R_DIR, REV);
      right_pwr = abs(right_pwr);
    } else {
      digitalWrite(R_DIR, FWD);
    }

    analogWrite(L_PWM, left_pwr);
    analogWrite(R_PWM, right_pwr);
  }

  void startRotating(float rotate_speed) {
    is_rotating = true;
    setPWM(rotate_speed, -rotate_speed);
  }

  void stopRotating() {
    if (is_rotating) {
      setPWM(0, 0);
      is_rotating = false;
    }
  }
};

#endif
