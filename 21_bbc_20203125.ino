// Arduino pin assignment
#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100
#define _DIST_MAX 450
#define _DIST_ALPHA 0.4


int a, b; // unit: mm
float c; 
float timeout; // unit: us
float dist_min, dist_max, dist_ema, alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms

Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
    myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(1470);
  delay(500);

  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; 
// initialize serial port
  Serial.begin(57600);

  last_sampling_time = 0;

  a = 69;
  b = 289;
}

float ir_distance(void){ 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  float raw_dist = ir_distance();
  dist_ema = (alpha) * raw_dist + (1-alpha) * dist_ema;
  float dist_cali = 100 + 300.0 / (b - a) * (dist_ema - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali > 255){
   c=1470+(255-dist_cali)*(200/145);
   myservo.writeMicroseconds(c);
   }
  else {
   c=1470+(255-dist_cali)*(200/155);
   myservo.writeMicroseconds(c);
  }
}
