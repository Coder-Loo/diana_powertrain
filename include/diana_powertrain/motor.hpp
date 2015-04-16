#ifndef MOTOR_HPP
#define MOTOR_HPP

template <class T> class Motor {
  Motor(hlcanopen::CanOpenManager<T> canOpenManager) {}
  ~Motor() {}

  void enable();
  void disable();
  void start();
  void setSpeed(int speed);
  int getSpeed();

};


#endif // MOTOR_HPP
