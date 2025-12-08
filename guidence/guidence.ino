#include <Arduino.h> // Don't know if neccesary, a lot of example code includes this.
#include <Arduino_BMI270_BMM150.h> // IMU
#include <Arduino_LPS22HB.h> // Barometer
#include <cmath> // Exponentation

const float targetAltitude = 750 / 3.281; // Convert feet target to meters

bool hasLaunched = false; // Recalibrate orientation at launch.
bool hasMECO = false; // Avoid activating the brakes during launch to avoid odd Apogee calculations.
bool hasApogee = false; // Switch code after Apogee to target landing time

int brakePin = 9; // PWM pin connected to brake servo

float gravity = 9.81; // Value of gravity

int hasLaunchedCheck = 0; // Check that we have postive velocity for a few frames before declaring hasLaunched.
int hasMECOCheck = 0; // Check that we have negative acceleration for a few frames before declaring hasMECO.
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

// How strong our feedForward control should be.
float feedForward = 1;

// miliseconds per frame
int mspf = 20;

float getAltitude() {
  // Barometric altitude.
  float pressure = BARO.readPressure();
  return (float) 44330*(1-pow(pressure / groundPressure, 1/5.255)); // https://physics.stackexchange.com/questions/333475/how-to-calculate-altitude-from-current-temperature-and-pressure
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

void setBrakes(float value) {
  // Value between 0.0 and 1.0 scaled to 0-255 for PWM
  // Note: May have to use CFD to get drag values for a bunch of different orientations and drag brake deployments, and then use that to calculate how much we need to open the brakes to get the amount of drag we want.
  // However, the force on the brakes is probaby about equal to the amount of drag we need, so we might be able to just do a small trig. calculation to get the amount of drag we need.
  // However, the orientation of the rocket will change the strength of the drag brakes.
  int pwm = (int) (value * 255);
  analogWrite(brakePin, pwm);
}

// Power up system right before launch.
void setup() {
  BARO.begin();
  groundPressure = BARO.readPressure(); // Set pressure here to avoid potential interference from the motor.

  IMU.begin();
  pinMode(brakePin, OUTPUT);
}

// Project our altitude and velocity outward to get our Apogee
float getApogee(float calAltitude, float calVelocity) {
  // Look for when the derivative (velocity) of the graph of our altitude is 0.
  // Issue with looking for derivative is that the function A\left(x\right) = \frac{gt^{2}}{2}+v_{i}t+A^{'}\left(x\right), which is a differiential equation; and I don't want to deal with that.
  return 0;
}

void loop() {
  float altitude = getAltitude();
  updateVelocity();
  updateAcceleration();
  float apogee = getApogee();

  float brakeTarget = 0; // The amount of braking we need to target this frame.

  if (hasLaunched) { // Want to avoid activating brakes until we launch to save on power (Also so we can check when the brakes randomly deploy).
    if (hasMECO) { // Want to avoid activating brakes until after launch
      if (!hasApogee) {
        // Path up towards apogee but before we reach apogee.
        if (altitude < targetAltitude) {
          float error = targetAltitude - apogee;
          float brakeValue = error * feedForward; //Since our error is the amount of drag we need, we should be able to just avoid using trig. because the push on the panel is the amount of drag we need.
          if (brakeValue > 1.0) brakeValue = 1.0;
          if (brakeValue < 0.0) brakeValue = 0.0;
          brakeTarget = brakeValue;
        } else {
          hasApogee = true;
          brakeTarget = 0; // retract brakes
        }
      } else {
        //Path downward
        BARO.end();
      }
    } else {
      if (accelerationLocalY < 9.75) { // Small value to avoid accidental activation before launch.
        hasMECOCheck++;
      } else {
        hasMECOCheck = 0;
      }

      if (hasMECOCheck >= 5) { // Wait for a few consecutive frames to activate guidence.
        hasMECO = true;
      }
      brakeTarget = 0;
    }
  } else {
    if (accelerationLocalY > 50) { // Small value to avoid accidental activation before launch. Calculated acceleration is 58 N / 0.53 kg - 9.81 m/s^2 = 99.6 m/s^2.
      hasLaunchedCheck++;
    } else {
      hasLaunchedCheck = 0;
    }

    if (hasLaunchedCheck >= 5) { // Wait for a few consecutive frames to activate launch flag.
      hasLaunched = true;

      // Reset velocity values to 0 due to the intergration.
      velocityLocalX = 0.0;
      velocityLocalY = 10; // Should start at 10 m/s because we are waiting 100 ms.
      velocityLocalZ = 0.0;

      // Reset orientation values due to Earth's rotation.
      rotationX = 0.0;
      rotationY = 0.0;
      rotationZ = 0.0;
    }
    brakeTarget = 0;
  }

  setBrakes(brakeTarget); // Avoid setting the brakes multiple times in 1 frame to maybe avoid bugs.
  delay(mspf); // small loop delay, decreased from 50 to 20 to decrease activation time from 250 ms to 100 ms
}
