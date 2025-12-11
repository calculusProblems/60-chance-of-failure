#include <Arduino.h> // Don't know if neccesary, a lot of example code includes this.
#include <Arduino_BMI270_BMM150.h> // IMU
#include <Arduino_LPS22HB.h> // Barometer
#include <cmath> // Exponentation

//Small program to check that the rotation and acceleration is actually working on the guidence.ino; copy getAltitude, updateVelocity, updateAcceleration, and updateOrientation from there.

float groundPressure = 0; // Used to calibrate the barometric altimeter

// Need to figure out if X, Y, or Z is up (use Y for now).
// For some reason, Vector3s are not supported.
// REMEMBER, THESE ARE LOCAL VELOCITY AND ACCELERATIONS, we will need to find world velocity and accelerations later.
float velocityLocalX = 0.0;
float velocityLocalY = 0.0;
float velocityLocalZ = 0.0;

float accelerationLocalX = 0.0;
float accelerationLocalY = 0.0;
float accelerationLocalZ = 0.0;

float rotationX = 0.0;//Ew Euler angles
float rotationY = 0.0;
float rotationZ = 0.0;

float velocityWorldX = 0.0;
float velocityWorldY = 0.0;
float velocityWorldZ = 0.0;

float accelerationWorldX = 0.0;
float accelerationWorldY = 0.0;
float accelerationWorldZ = 0.0;

// miliseconds per frame
int mspf = 20;

float BAROGetAltitude() {
  // Barometric altitude.
  float pressure = BARO.readPressure();
  return (float) 44330*(1-pow(pressure / groundPressure, 1/5.255)); // https://physics.stackexchange.com/questions/333475/how-to-calculate-altitude-from-current-temperature-and-pressure
}

float actGetAltitude() {
  // Barometric altitude.
  // float pressure = BARO.readPressure();
  // return (float) 44330*(1-pow(pressure / groundPressure, 1/5.255)); // https://physics.stackexchange.com/questions/333475/how-to-calculate-altitude-from-current-temperature-and-pressure

  // Actual altimeter
  int total = 0;
  for (int i = 0; i < (int) sizeof(altimeterPins)/sizeof(int); i++) total += digitalRead(altimeterPins[i]) * pow(2, i);
  return total;
}

void updateVelocity() {
  // Replace with IMU velocity reading or derivative of altitude.
  // Need to get different axii because drag = velocity^2, so we can't just calculate one-dimension.
  velocityLocalX += accelerationLocalX * mspf / 1000; // Want to try integrating acceleration, might drift, but since we only need it to work for ~5 sec, it should be fine.
  velocityLocalY += accelerationLocalY * mspf / 1000;
  velocityLocalZ += accelerationLocalZ * mspf / 1000;
  
  // Do some magic to update velocityWorld based on velocityLocal here.
}

void updateAcceleration() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelerationLocalX, accelerationLocalY, accelerationLocalZ);
  }
  // Do some magic to update accelerationWorld here.
  // Also need to know which axii for the rotation are first.
}

void updateOrientation() {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(rotationX, rotationY, rotationZ);
  }
}

// Power up system right before launch.
void setup() {
  Serial.print("Initializing BARO");
  BARO.begin();
  if (!BARO.begin()) {
    Serial.print("BARO initialization failed");
    while (1);
  }

  Serial.print("Initializing IMU");
  IMU.begin();
  if (!IMU.begin()) {
    Serial.print("IMU initialization failed");
    while (1);
  }
  
  Serial.print("IMU acceleration sampling rate:");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  
  Serial.print("IMU rotation sampling rate:");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  groundPressure = BARO.readPressure(); // Set pressure here to avoid potential interference from the motor.
  
  for (int i = 0; i < (int) sizeof(altimeterPins)/sizeof(int); i++) pinMode(altimeterPins[i], INPUT);
}

// Project our altitude and velocity outward to get our Apogee
float getApogee() {
  // Look for when the derivative (velocity) of the graph of our altitude is 0.
  // Issue with looking for derivative is that the function A\left(x\right) = \frac{gt^{2}}{2}+v_{i}t+A^{'}\left(x\right), which is a differiential equation; and I don't want to deal with that.
  return 0;
}

void loop() {
  float BAROAltitude = BAROGetAltitude();
  float actAltitude = actGetAltitude();
  updateVelocity();
  updateAcceleration();

  Serial.print("BARO Altitude: ");
  Serial.print(BAROAltitude);
  
  Serial.print("\n\n");

  Serial.print("Act Altitude: ");
  Serial.print(BAROAltitude);
  
  Serial.print("\n\n");

  Serial.print("Local Velocity (x,y,z): ");
  Serial.print(velocityLocalX);
  Serial.print(", ");
  Serial.print(velocityLocalY);
  Serial.print(", ");
  Serial.print(velocityLocalZ);

  Serial.print("\n\n");

  Serial.print("World Velocity (x,y,z): ");
  Serial.print(velocityWorldX);
  Serial.print(", ");
  Serial.print(velocityWorldY);
  Serial.print(", ");
  Serial.print(velocityWorldZ);

  Serial.print("\n\n");

  Serial.print("Local Acceleration (x,y,z): ");
  Serial.print(accelerationLocalX);
  Serial.print(", ");
  Serial.print(accelerationLocalY);
  Serial.print(", ");
  Serial.print(accelerationLocalZ);

  Serial.print("\n\n");

  Serial.print("World Acceleration (x,y,z): ");
  Serial.print(accelerationWorldX);
  Serial.print(", ");
  Serial.print(accelerationWorldY);
  Serial.print(", ");
  Serial.print(accelerationWorldZ);

  Serial.print("\n\n");

  Serial.print("Rotation (x,y,z): ");
  Serial.print(rotationX);
  Serial.print(", ");
  Serial.print(rotationY);
  Serial.print(", ");
  Serial.print(rotationZ);

  delay(mspf); // small loop delay, decreased from 50 to 20 to decrease activation time from 250 ms to 100 ms
}
