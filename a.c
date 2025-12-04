const float targetAltitude = 500; // target in meters
bool reachedTarget = false;
int finPin = 9; // PWM pin connected to fin servo

float readAltitude() {
  // Replace with your actual altitude sensor reading
  return 0;
}

void setFins(float value) {
  // value between 0.0 and 1.0 scaled to 0-255 for PWM
  int pwm = int(value * 255);
  analogWrite(finPin, pwm);
}

void setup() {
  pinMode(finPin, OUTPUT);
}

void loop() {
  float altitude = readAltitude();

  if (!reachedTarget) {
    if (altitude < targetAltitude) {
      float diff = targetAltitude - altitude;
      float finValue = diff / targetAltitude;
      if (finValue > 1.0) finValue = 1.0;
      if (finValue < 0.0) finValue = 0.0;
      setFins(finValue);
    } else {
      reachedTarget = true;
      setFins(0); // retract fins
    }
  } else {
    setFins(0); // fins stay retracted
  }
  
  delay(50); // small loop delay
}
