#include "EjeMotor.h"

EjeMotor::EjeMotor(String n, int pul, int dir, int en, int stepsRev)
  : name(n), pulPin(pul), dirPin(dir), enablePin(en), stepsPerRevolution(stepsRev) {}

void EjeMotor::move(int position) {
  targetPosition = position;
  if (position == 0) {
    doHoming();
  } else {
    moveSelectedPosition(position);
  }
}

void EjeMotor::moveSelectedPosition(int position) {
  if (position > currentPosition) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }
  isRunning = true;
  Serial.print(name + " moviendo a: ");
  Serial.println(targetPosition);
}

void EjeMotor::updateMotor() {
  currentMicros = micros();
  if (currentMicros - previousMicros >= pulseDuration) {
    previousMicros = currentMicros;

    Serial.print(name + ": ");
    Serial.println(currentPosition);
    if (currentPosition != targetPosition) {
      pulseState = !pulseState;
      digitalWrite(pulPin, pulseState);

      if (pulseState == LOW) {
        if (digitalRead(dirPin) == HIGH) {
          currentPosition++;
        } else {
          currentPosition--;
        }
      }
    } else {
      isRunning = false;
      Serial.print(name + " posición alcanzada: ");
      Serial.println(currentPosition);
    }
  }
}

void EjeMotor::doHoming() {
  Serial.println(name + ": Iniciando secuencia de homing ...");
  int originalSpeed = motorSpeed;
  motorSpeed = velocidad * 4;
  pulseDuration = velocidad;

  targetPosition = 0;
  isRunning = true;
  if (targetPosition > currentPosition) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  Serial.print(name + ": Moviendo a posición: ");
  Serial.println(targetPosition);

  motorSpeed = originalSpeed;
  pulseDuration = velocidad;

  currentPosition = 0;
  targetPosition = 0;
  Serial.println(name + ": Homing completado. Posición establecida a 0.");
}
