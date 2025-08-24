#ifndef EJEMOTOR_H
#define EJEMOTOR_H

#include <Arduino.h>

struct EjeMotor {
  const String name;
  const int pulPin;
  const int dirPin;
  const int enablePin;
  const int stepsPerRevolution;

  unsigned long previousMicros = 0;
  unsigned long currentMicros = 0;
  unsigned long pulseDuration = 50;
  bool pulseState = LOW;
  int currentPosition = 0;
  int targetPosition = 0;
  bool isRunning = false;
  int velocidad = 50;
  int motorSpeed = 1000;

  EjeMotor(String n, int pul, int dir, int en, int stepsRev);

  void move(int position);
  void moveSelectedPosition(int position);
  void updateMotor();
  void doHoming();
};

#endif
