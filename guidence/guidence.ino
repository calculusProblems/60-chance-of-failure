#include <Arduino.h> // Don't know if neccesary, a lot of example code includes this.
#include <Arduino_BMI270_BMM150.h> // IMU
#include <Arduino_LPS22HB.h> // Barometer
#include <Vector.h>

const float targetAltitude = 750 / 3.281; // Convert feet target to meters

bool hasApogee = false; // Switch code after Apogee to target landing time
bool hasLaunched = false; // Avoid activating the brakes during launch to avoid odd Apogee calculations.

float brakeTarget = 0;

int brakePin = 9; // PWM pin connected to brake servo

float gravity = 9.81; // Value of gravity

int gravAcceleration = 0; // Check that we have negative acceleration for a few frames before declaring hasLaunched.

float getAltitude() {
  // Replace with IMU or barometer altitude reading
  return (float) 0;
}

Vector getVelocity() {
  // Replace with IMU velocity reading or derivative of altitude.
  // Need Vector3 with velocity because drag = velocity^2
  return (Vector) (0, 0, 0);
}

float getAcceleration() {
  // Replace with IMU acceleration reading, derivative of velocity.
  return (float) 0;
}

void setBrakes(float value) {
  // Value between 0.0 and 1.0 scaled to 0-255 for PWM
  int pwm = (int) (value * 255);
  analogWrite(brakePin, pwm);
}

void setup() {
  BARO.begin();
  pinMode(brakePin, OUTPUT);
}

//Project our altitude and velocity outward to get our Apogee
float getApogee(float calAltitude, float calVelocity) {
  //Look for when the derivative (velocity) of the graph of our altitude is 0.
  //Issue with looking for derivative is that the function A\left(x\right) = \frac{gt^{2}}{2}+v_{i}t+A^{'}\left(x\right), which is a differiential equation.
  return 0;
}

void loop() {
  float altitude = getAltitude();
  Vector velocity = getVelocity();
  float acceleration = getAcceleration();
  //float apogee = getApogee();

  if (hasLaunched) {//Want to avoid activating brakes until after launch
    if (!hasApogee) {
      if (altitude < targetAltitude) {
        float diff = targetAltitude - altitude;
        float brakeValue = diff / targetAltitude;
        if (brakeValue > 1.0) brakeValue = 1.0;
        if (brakeValue < 0.0) brakeValue = 0.0;
        brakeTarget = brakeValue;
      } else {
        hasApogee = true;
        brakeTarget = 0; // retract brakes
      }
    } else {
      brakeTarget = 0; // brakes stay retracted
    }
  } else {
    if (acceleration < 0) {

    }
    brakeTarget = 0;
  }

  setBrakes(brakeTarget);//Avoid setting the brakes multiple times in 1 frame to avoid bugs.
  delay(50); // small loop delay
}
