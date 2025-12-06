#include <Arduino.h> // Don't know if neccesary, a lot of example code includes this.
#include <Arduino_BMI270_BMM150.h> // IMU
#include <Arduino_LPS22HB.h> // Barometer
#include <cmath> // Exponentation

const float targetAltitude = 750 / 3.281; // Convert feet target to meters

bool hasLaunched = false; // Recalibrate orientation at launch.
bool hasMECO = false; // Avoid activating the brakes during launch to avoid odd Apogee calculations.
bool hasApogee = false; // Switch code after Apogee to target landing time

float brakeTarget = 0;

int brakePin = 9; // PWM pin connected to brake servo

float gravity = 9.81; // Value of gravity

int hasLaunchedCheck = 0; // Check that we have postive velocity for a few frames before declaring hasLaunched.
int hasMECOCheck = 0; // Check that we have negative acceleration for a few frames before declaring hasMECO.
float groundPressure = 0;//Used to calibrate the barometric altimeter

//Need to figure out if X, Y, or Z is up (use Y for now).
//For some reason, Vector3s are not supported.
//REMEMBER, THESE ARE LOCAL VELOCITY AND ACCELERATIONS, we will need to find world velocity and accelerations later.
float velocityX = 0.0;
float velocityY = 0.0;
float velocityZ = 0.0;

float accelerationX = 0.0;
float accelerationY = 0.0;
float accelerationZ = 0.0;

float rotationX = 0.0;//Ew Euler angles
float rotationY = 0.0;
float rotationZ = 0.0;

float getAltitude() {
  //Barometric altitude.
  float pressure = BARO.readPressure();
  return (float) 44330*(1-pow(pressure / groundPressure, 1/5.255)); // https://physics.stackexchange.com/questions/333475/how-to-calculate-altitude-from-current-temperature-and-pressure
}

void updateVelocity() {
  // Replace with IMU velocity reading or derivative of altitude.
  // Need to get different axii because drag = velocity^2, so we can't just calculate one-dimension.
  velocityX += accelerationX; // Want to try integrating acceleration, might drift, but since we only need it to work for ~5 sec, it should be fine.
  velocityY += accelerationY;
  velocityZ += accelerationZ;
}

void updateAcceleration() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelerationX, accelerationY, accelerationZ);
  }
}

void updateOrientation() {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(rotationX, rotationY, rotationZ);
  }
}

void setBrakes(float value) {
  // Value between 0.0 and 1.0 scaled to 0-255 for PWM
  // Note: May have to use CFD to get drag values for a bunch of different orientations and drag brake deployments, and then use that to calculate how much we need to open the brakese to get the amount of drag we want.
  // However, the force on the brakes is probaby about equal to the amount of drag we need, so we might be able to just do a small trig. calculation to get the amount of drag we need.
  // However, the orientation of the rocket will change the strength of the drag brakes.
  int pwm = (int) (value * 255);
  analogWrite(brakePin, pwm);
}

//Power up system right before launch.
void setup() {
  BARO.begin();
  groundPressure = BARO.readPressure(); // Set pressure here to avoid potential interference from the motor.

  IMU.begin();
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
  updateVelocity();
  updateAcceleration();
  //float apogee = getApogee();

  if (hasLaunched) {
  if (hasMECO) {//Want to avoid activating brakes until after launch
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

      BARO.end();
    }
  } else {
    if (accelerationY < 9.75) {// Small value to avoid accidental activation before launch.
      hasMECOCheck++;
    } else {
      hasMECOCheck = 0;
    }

    if (hasMECOCheck >= 5) {
      hasMECO = true;
    }
    brakeTarget = 0;
  }
  } else {
    if (accelerationY > 0.05) {// Small value to avoid accidental activation before launch.
      hasLaunchedCheck++;
    } else {
      hasLaunchedCheck = 0;
    }

    if (hasLaunchedCheck >= 5) {
      hasLaunched = true;

      //Reset velocity values to 0 due to the intergration.
      velocityX = 0.0;
      velocityY = 0.0;
      velocityZ = 0.0;

      //Reset orientation values due to Earth's rotation.
      rotationX = 0.0;
      rotationY = 0.0;
      rotationZ = 0.0;
    }
    brakeTarget = 0;
  }

  setBrakes(brakeTarget);//Avoid setting the brakes multiple times in 1 frame to maybe avoid bugs.
  delay(20); // small loop delay, decreased from 50 to 20 to decrease activation time from 250 ms to 100 ms
}
