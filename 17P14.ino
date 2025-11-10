#include <Servo.h>

#define PIN_IR    A0
#define PIN_LED   9
#define PIN_SERVO 10

// Servo pulse (us)
#define _DUTY_MIN 500
#define _DUTY_NEU 1500
#define _DUTY_MAX 2500

// Distance range (mm) : 10cm ~ 25cm
#define _DIST_MIN 100.0
#define _DIST_MAX 250.0

// Filters & loop
#define EMA_ALPHA  0.3
#define LOOP_INTERVAL 25   // ms (>= 20ms)

Servo myservo;
unsigned long last_loop_time = 0;

float dist_prev = _DIST_MIN;
float dist_ema  = _DIST_MIN;

void setup() {
  pinMode(PIN_LED, OUTPUT);

  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);
  myservo.writeMicroseconds(_DUTY_NEU);

  Serial.begin(1000000);  
}

void loop() {
  unsigned long now = millis();
  if (now < last_loop_time + LOOP_INTERVAL) return;
  last_loop_time += LOOP_INTERVAL;

  int   ir_adc   = analogRead(PIN_IR);   
  float a_value  = (float)ir_adc;

  float dist_raw;
  if (a_value <= 9.5f) {
    dist_raw = _DIST_MAX + 100.0f;
  } else {
    dist_raw = (6762.0f / (a_value - 9.0f) - 4.0f) * 10.0f - 60.0f; // mm
  }

 
  float dist_valid;
  if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX) {
    dist_valid = dist_raw;
    digitalWrite(PIN_LED, HIGH);  
    dist_prev = dist_raw;
  } else {
    dist_valid = dist_prev;        
    digitalWrite(PIN_LED, LOW);
  }

  dist_ema = EMA_ALPHA * dist_valid + (1.0f - EMA_ALPHA) * dist_ema;

  float in_span   = (_DIST_MAX - _DIST_MIN);      
  float t         = (dist_ema - _DIST_MIN) / in_span;   
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  float angle_f   = t * 180.0f;                  

  float duty_f = _DUTY_MIN + (angle_f / 180.0f) * (float)(_DUTY_MAX - _DUTY_MIN);
  if (duty_f < _DUTY_MIN) duty_f = _DUTY_MIN;
  if (duty_f > _DUTY_MAX) duty_f = _DUTY_MAX;
  int duty = (int)(duty_f + 0.5f);

  myservo.writeMicroseconds(duty);

  Serial.print("IR:");       Serial.print(a_value);
  Serial.print(",dist_raw:");Serial.print(dist_raw);
  Serial.print(",ema:");     Serial.print(dist_ema);
  Serial.print(",angle:");   Serial.print(angle_f);
  Serial.print(",servo:");   Serial.print(duty);
  Serial.println();
}
