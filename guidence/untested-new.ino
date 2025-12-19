#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_LPS22HB.h>
#include <cmath>

const float targetAltitude = 750 / 3.281;

bool hasLaunched = false;
bool hasMECO = false;
bool hasApogee = false;

int brakePins[] = {9, 10};
int altimeterPins[] = {2, 3, 4, 5, 6, 7};

const float gravity = 9.81;
const float rho = 1.225;
const float Cd = 1.2;
const float area = 0.01;
const float mass = 0.53;

int hasLaunchedCheck = 0;
int hasMECOCheck = 0;

float groundPressure = 0;

float velocityLocalX = 0.0;
float velocityLocalY = 0.0;
float velocityLocalZ = 0.0;

float accelerationLocalX = 0.0;
float accelerationLocalY = 0.0;
float accelerationLocalZ = 0.0;

float rotationX = 0.0;
float rotationY = 0.0;
float rotationZ = 0.0;

float feedForward = 1;
int mspf = 20;

float getAltitude() {
  int total = 0;
  for (int i = 0; i < (int)(sizeof(altimeterPins) / sizeof(int)); i++) {
    total += digitalRead(altimeterPins[i]) * (1 << i);
  }
  return total;
}

void updateVelocity() {
  velocityLocalY += accelerationLocalY * (mspf / 1000.0);
}

void updateAcceleration() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelerationLocalX, accelerationLocalY, accelerationLocalZ);
  }
}

void updateOrientation() {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(rotationX, rotationY, rotationZ);
  }
}

void setBrakes(float value) {
  int pwm = (int)(value * 255);
  analogWrite(brakePins[0], pwm);
  analogWrite(brakePins[1], pwm);
}

void setup() {
  BARO.begin();
  IMU.begin();

  groundPressure = BARO.readPressure();

  pinMode(brakePins[0], OUTPUT);
  pinMode(brakePins[1], OUTPUT);
  for (int i = 0; i < (int)(sizeof(altimeterPins) / sizeof(int)); i++) {
    pinMode(altimeterPins[i], INPUT);
  }
}

float getApogee() {
  float v = velocityLocalY;
  if (v <= 0) return getAltitude();

  float k = 0.5 * rho * Cd * area / mass;
  float term = (gravity + k * v * v) / gravity;
  float deltaH = (1.0 / (2.0 * k)) * log(term);

  return getAltitude() + deltaH;
}

void loop() {
  float altitude = getAltitude();
  updateAcceleration();
  updateVelocity();

  float apogee = getApogee();
  float brakeTarget = 0;

  if (hasLaunched) {
    if (hasMECO) {
      if (!hasApogee) {
        if (apogee < targetAltitude) {
          float error = targetAltitude - apogee;
          float brakeValue = error * feedForward;
          if (brakeValue > 1.0) brakeValue = 1.0;
          if (brakeValue < 0.0) brakeValue = 0.0;
          brakeTarget = brakeValue;
        } else {
          hasApogee = true;
          brakeTarget = 0;
        }
      }
    } else {
      if (accelerationLocalY < 9.75) hasMECOCheck++;
      else hasMECOCheck = 0;
      if (hasMECOCheck >= 5) hasMECO = true;
    }
  } else {
    if (accelerationLocalY > 50) hasLaunchedCheck++;
    else hasLaunchedCheck = 0;

    if (hasLaunchedCheck >= 5) {
      hasLaunched = true;
      velocityLocalY = 10.0;
      rotationX = rotationY = rotationZ = 0.0;
    }
  }

  setBrakes(brakeTarget);
  delay(mspf);
}
