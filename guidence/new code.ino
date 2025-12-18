#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_LPS22HB.h>
#include <cmath>

const float targetAltitude = 750 / 3.281;

const int brakePins[] = {9, 10};
const int altimeterPins[] = {2, 3, 4, 5, 6, 7};

const int mspf = 20;
const float feedForward = 0.002;

bool hasLaunched = false;
bool hasMECO = false;
bool hasApogee = false;

int hasLaunchedCheck = 0;
int hasMECOCheck = 0;

float accelerationLocalX = 0;
float accelerationLocalY = 0;
float accelerationLocalZ = 0;

float velocityLocalY = 0;

float startTime = 0;

const int FIT_N = 8;
float fitT[FIT_N];
float fitH[FIT_N];
int fitIndex = 0;
bool fitFull = false;

float getAltitude() {
  int total = 0;
  for (int i = 0; i < (int)(sizeof(altimeterPins) / sizeof(int)); i++) {
    total += digitalRead(altimeterPins[i]) * (1 << i);
  }
  return total;
}

void updateAcceleration() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelerationLocalX, accelerationLocalY, accelerationLocalZ);
  }
}

void updateVelocity() {
  velocityLocalY += accelerationLocalY * (mspf / 1000.0);
}

void pushFitSample(float t, float h) {
  fitT[fitIndex] = t;
  fitH[fitIndex] = h;
  fitIndex++;
  if (fitIndex >= FIT_N) {
    fitIndex = 0;
    fitFull = true;
  }
}

bool estimateApogee(float &apogeeOut) {
  int n = fitFull ? FIT_N : fitIndex;
  if (n < 5) return false;

  float S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
  float T0 = 0, T1 = 0, T2 = 0;

  for (int i = 0; i < n; i++) {
    float t = fitT[i];
    float h = fitH[i];
    float t2 = t * t;
    S0 += 1;
    S1 += t;
    S2 += t2;
    S3 += t2 * t;
    S4 += t2 * t2;
    T0 += h;
    T1 += h * t;
    T2 += h * t2;
  }

  float D = S0*(S2*S4 - S3*S3) - S1*(S1*S4 - S2*S3) + S2*(S1*S3 - S2*S2);
  if (fabs(D) < 1e-6) return false;

  float a = (T0*(S2*S4 - S3*S3) - S1*(T1*S4 - S3*T2) + S2*(T1*S3 - S2*T2)) / D;
  float b = (S0*(T1*S4 - S3*T2) - T0*(S1*S4 - S2*S3) + S2*(S1*T2 - T1*S2)) / D;
  float c = (S0*(S2*T2 - T1*S3) - S1*(S1*T2 - T1*S2) + T0*(S1*S3 - S2*S2)) / D;

  if (a >= 0) return false;

  float tPeak = -b / (2 * a);
  apogeeOut = a * tPeak * tPeak + b * tPeak + c;
  return true;
}

void setBrakes(float value) {
  if (value < 0) value = 0;
  if (value > 1) value = 1;
  int pwm = (int)(value * 255);
  analogWrite(brakePins[0], pwm);
  analogWrite(brakePins[1], pwm);
}

void setup() {
  startTime = millis();
  Serial.begin(115200);

  IMU.begin();

  pinMode(brakePins[0], OUTPUT);
  pinMode(brakePins[1], OUTPUT);
  for (int i = 0; i < (int)(sizeof(altimeterPins) / sizeof(int)); i++) {
    pinMode(altimeterPins[i], INPUT);
  }
}

void loop() {
  float t = (millis() - startTime) / 1000.0;
  updateAcceleration();
  updateVelocity();

  float altitude = getAltitude();
  pushFitSample(t, altitude);

  float brakeTarget = 0;

  if (!hasLaunched) {
    if (accelerationLocalY > 50) hasLaunchedCheck++;
    else hasLaunchedCheck = 0;
    if (hasLaunchedCheck >= 5) {
      hasLaunched = true;
      velocityLocalY = 10;
    }
  } else if (!hasMECO) {
    if (accelerationLocalY < 9.75) hasMECOCheck++;
    else hasMECOCheck = 0;
    if (hasMECOCheck >= 5) hasMECO = true;
  } else if (!hasApogee) {
    float predictedApogee;
    if (estimateApogee(predictedApogee)) {
      float error = predictedApogee - targetAltitude;
      brakeTarget = error * feedForward;
      if (error <= 0) hasApogee = true;
    }
  }

  setBrakes(brakeTarget);
  delay(mspf);
}
