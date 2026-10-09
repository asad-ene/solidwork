#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();

#define servo1 0  // base
#define servo2 1  // shoulder
#define servo3 2  // elbow
#define servo4 3  // gripper

// PWM limits (adjust later)
#define PWM_MIN 150
#define PWM_MAX 450

float L1 = 100.0;
float L2 = 100.0;

// ---------- PWM <-> Angle conversions ----------
float pwmToAngle(int pwm) {
  return map(pwm, PWM_MIN, PWM_MAX, 0, 180);
}

int angleToPWM(float angleDeg) {
  // Clamp angle to valid servo range before mapping
  angleDeg = constrain(angleDeg, 0, 180);
  return (int)map(angleDeg, 0, 180, PWM_MIN, PWM_MAX);
}

// ---------- Forward Kinematics ----------
void computeFK(float t1, float t2, float t3) {
  float th1 = radians(t1);
  float th2 = radians(t2);
  float th3 = radians(t3);

  float r = L1 * cos(th2) + L2 * cos(th2 + th3);
  float z = L1 * sin(th2) + L2 * sin(th2 + th3);

  float x = r * cos(th1);
  float y = r * sin(th1);

  Serial.print("X: "); Serial.print(x);
  Serial.print("  Y: "); Serial.print(y);
  Serial.print("  Z: "); Serial.println(z);
}

// ---------- Inverse Kinematics ----------
// Given a target (x, y, z), solve for joint angles (deg).
// Returns false if the point is unreachable.
// elbowUp: choose one of the two valid elbow solutions.
bool computeIK(float x, float y, float z, float &t1, float &t2, float &t3, bool elbowUp = true) {
  // Base rotation
  float th1 = atan2(y, x);

  // Project target into the arm's vertical (r, z) plane
  float r = sqrt(x * x + y * y);

  float dist2 = r * r + z * z;
  float maxReach = L1 + L2;
  float minReach = fabs(L1 - L2);

  // Reachability check
  if (sqrt(dist2) > maxReach || sqrt(dist2) < minReach) {
    Serial.println("IK Error: Target unreachable");
    return false;
  }

  // Elbow angle via law of cosines
  float cosTh3 = (dist2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  cosTh3 = constrain(cosTh3, -1.0, 1.0);  // guard against float rounding
  float sinTh3 = sqrt(1.0 - cosTh3 * cosTh3);
  if (!elbowUp) sinTh3 = -sinTh3;

  float th3 = atan2(sinTh3, cosTh3);

  // Shoulder angle
  float th2 = atan2(z, r) - atan2(L2 * sinTh3, L1 + L2 * cosTh3);

  // Convert back to degrees
  t1 = degrees(th1);
  t2 = degrees(th2);
  t3 = degrees(th3);

  return true;
}

// Moves the arm to a target (x, y, z) using IK, if reachable
void moveToXYZ(float x, float y, float z, bool elbowUp = true) {
  float t1, t2, t3;
  if (!computeIK(x, y, z, t1, t2, t3, elbowUp)) return;

  int pwm1 = angleToPWM(t1);
  int pwm2 = angleToPWM(t2);
  int pwm3 = angleToPWM(t3);

  srituhobby.setPWM(servo1, 0, pwm1);
  srituhobby.setPWM(servo2, 0, pwm2);
  srituhobby.setPWM(servo3, 0, pwm3);

  Serial.print("IK -> t1: "); Serial.print(t1);
  Serial.print("  t2: "); Serial.print(t2);
  Serial.print("  t3: "); Serial.println(t3);
}

void setup() {
  Serial.begin(9600);
  srituhobby.begin();
  srituhobby.setPWMFreq(60);

  srituhobby.setPWM(servo1, 0, 330);
  srituhobby.setPWM(servo2, 0, 150);
  srituhobby.setPWM(servo3, 0, 300);
  srituhobby.setPWM(servo4, 0, 410);

  delay(3000);
}

void loop() {

  for (int S1value = 330; S1value >= 250; S1value--) {
    srituhobby.setPWM(servo1, 0, S1value);

    float t1 = pwmToAngle(S1value);
    float t2 = pwmToAngle(150);
    float t3 = pwmToAngle(300);

    computeFK(t1, t2, t3);
    delay(10);
  }

  for (int S2value = 150; S2value <= 380; S2value++) {
    srituhobby.setPWM(servo2, 0, S2value);

    float t1 = pwmToAngle(250);
    float t2 = pwmToAngle(S2value);
    float t3 = pwmToAngle(300);

    computeFK(t1, t2, t3);
    delay(10);
  }

  for (int S3value = 300; S3value <= 380; S3value++) {
    srituhobby.setPWM(servo3, 0, S3value);

    float t1 = pwmToAngle(250);
    float t2 = pwmToAngle(380);
    float t3 = pwmToAngle(S3value);

    computeFK(t1, t2, t3);
    delay(10);
  }

  delay(2000);

}