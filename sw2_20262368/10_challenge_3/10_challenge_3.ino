#include <Servo.h>
#include <math.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define DETECT_MM  100.0
#define RELEASE_MM 130.0
#define CONFIRM_N  3

// duty duration for myservo.writeMicroseconds()
// CALIBRATED VALUES (servo by servo) - 측정값으로 교체할 것
#define _DUTY_MIN 500     // servo 0 degree (measured)
#define _DUTY_MAX 2400    // servo 180 degree (measured)

#define ANGLE_DOWN 0.0
#define ANGLE_UP   90.0
#define MOVE_TIME   2000
#define CLOSE_DELAY 1500

int easingMode = 1;
#define SIGMOID_K 10.0

Servo myservo;

enum State { CLOSED, OPENING, OPEN, CLOSING };
State state = CLOSED;

float currentAngle = ANGLE_DOWN;
float moveFrom, moveTo;
unsigned long moveStart, moveDuration;

bool carPresent = false;
int  hitCount = 0, missCount = 0;
float dist_raw = 0;
unsigned long last_sampling_time = 0;
unsigned long carGoneTime = 0;

float easeLinear(float t) {
  return t;
}

float easeSigmoid(float t) {
  float s  = 1.0 / (1.0 + exp(-SIGMOID_K * (t - 0.5)));
  float s0 = 1.0 / (1.0 + exp( SIGMOID_K * 0.5));
  float s1 = 1.0 / (1.0 + exp(-SIGMOID_K * 0.5));
  return (s - s0) / (s1 - s0);
}

float easeCosine(float t) {
  return 0.5 - 0.5 * cos(PI * t);
}

float ease(float t) {
  switch (easingMode) {
    case 1:  return easeSigmoid(t);
    case 2:  return easeCosine(t);
    default: return easeLinear(t);
  }
}

void writeAngle(float angle) {
  int duty = _DUTY_MIN + angle * (_DUTY_MAX - _DUTY_MIN) / 180.0;
  myservo.writeMicroseconds(duty);
}

void startMove(float target) {
  moveFrom  = currentAngle;
  moveTo    = target;
  moveStart = millis();
  float frac = fabs(moveTo - moveFrom) / (ANGLE_UP - ANGLE_DOWN);
  moveDuration = (unsigned long)(MOVE_TIME * frac);
  if (moveDuration < 1) moveDuration = 1;
}

bool updateMove() {
  float t = (float)(millis() - moveStart) / moveDuration;
  if (t > 1.0) t = 1.0;
  currentAngle = moveFrom + (moveTo - moveFrom) * ease(t);
  writeAngle(currentAngle);
  return t >= 1.0;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}

void updateSensor() {
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  if (dist_raw == 0.0) {
    if (missCount < CONFIRM_N) missCount++;
    hitCount = 0;
  } else if (dist_raw < DETECT_MM) {
    if (hitCount < CONFIRM_N) hitCount++;
    missCount = 0;
  } else if (dist_raw > RELEASE_MM) {
    if (missCount < CONFIRM_N) missCount++;
    hitCount = 0;
  }

  if (!carPresent && hitCount >= CONFIRM_N) {
    carPresent = true;
    digitalWrite(PIN_LED, LOW);   // LED ON (active-low)
  }
  if (carPresent && missCount >= CONFIRM_N) {
    carPresent = false;
    carGoneTime = millis();
    digitalWrite(PIN_LED, HIGH);  // LED OFF (active-low)
  }
}

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);   // turn-off Sonar
  digitalWrite(PIN_LED, HIGH);   // LED OFF (active-low)

  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);
  writeAngle(ANGLE_DOWN);
  delay(500);

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '2') easingMode = c - '0';
  }

  switch (state) {
    case CLOSED:
      if (carPresent) { startMove(ANGLE_UP); state = OPENING; }
      break;

    case OPENING:
      if (updateMove()) state = OPEN;
      break;

    case OPEN:
      if (!carPresent && millis() - carGoneTime >= CLOSE_DELAY) {
        startMove(ANGLE_DOWN);
        state = CLOSING;
      }
      break;

    case CLOSING:
      if (carPresent) {
        startMove(ANGLE_UP);
        state = OPENING;
      } else if (updateMove()) {
        state = CLOSED;
      }
      break;
  }

  if (millis() - last_sampling_time < INTERVAL)
    return;
  last_sampling_time = millis();

  updateSensor();

  Serial.print("dist:");    Serial.print(dist_raw);
  Serial.print(",Angle:");  Serial.print(currentAngle);
  Serial.print(",Car:");    Serial.print(carPresent ? ANGLE_UP : ANGLE_DOWN);
  Serial.print(",Mode:");   Serial.print(easingMode * 10);
  Serial.println("");
}
