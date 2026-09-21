#define PIN_LED 7

//   10ms  -> 10000
//   1ms   ->  1000
//   0.1ms ->   100
#define PWM_PERIOD_US 100

int period = 10000;
int duty   = 0;

void set_period(int p) {
  if (p < 100)   p = 100;
  if (p > 10000) p = 10000;
  period = p;
}


void set_duty(int d) {
  if (d < 0)   d = 0;
  if (d > 100) d = 100;
  duty = d;
}


void pwm_cycle() {
  unsigned long on_time  = (unsigned long)period * duty / 100;
  unsigned long off_time = period - on_time;

  if (on_time > 0) {
    digitalWrite(PIN_LED, LOW);
    delayMicroseconds(on_time);
  }
  if (off_time > 0) {
    digitalWrite(PIN_LED, HIGH);
    delayMicroseconds(off_time);
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  set_period(PWM_PERIOD_US);
  set_duty(0);
}

void loop() {
  unsigned long t = millis() % 1000;
  int d;
  if (t < 500) d = t * 100 / 500;
  else         d = (1000 - t) * 100 / 500;
  set_duty(d);
  pwm_cycle();
}
