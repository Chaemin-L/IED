// Arduino pin assignment
#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9
#define INTERVAL 20 // sampling interval (unit: ms)

#define _DIST_ALPHA 0.35

// Framework setting
#define _DIST_TARGET 255  
#define _DIST_MIN 100
#define _DIST_MAX 400

// Servo range
#define _DUTY_MIN 1230
#define _DUTY_NEU 1430 
#define _DUTY_MAX 1630

// Servo speed control
#define _SERVO_SPEED 60      // servo 속도 설정
#define _SERVO_ANGLE 20 

// Event periods
#define _INTERVAL_DIST 20         // distance 측정 이벤트 주기
#define _INTERVAL_SERVO 20     // servo 조정 이벤트 주기
#define _INTERVAL_SERIAL 100   // serial 출력 이벤트 주기

// PID parameters
#define _KP_m 0.3
#define _KP_M 0.6

// global variables //
float dist_min, dist_max,  filtered_dist, alpha; 
// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

float error_curr, control, pterm;

void setup(){
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);


// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;

// move servo to neutral position
 myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE*2) * ((float)_INTERVAL_SERVO / 1000);
}


void loop() {
  dist_raw = ir_distance();
/////////////////////
// Event generator // 이벤트 실행 간격 구현 
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

  ////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
      event_dist = false; // 
  // get a distance reading from the distance sensor
      dist_ema = (alpha) * dist_raw + (1-alpha) * dist_ema;
      filtered_dist = ir_distance_filtered();    //적외선 센서 필터링 값 저장

  // PID control logic
    error_curr = _DIST_TARGET - filtered_dist; // 현재 오차 저장
    if (error_curr < 0){
      pterm = error_curr*_KP_M;
      }
    else{
      pterm = error_curr*_KP_m;
    }
    control = pterm;    // 제어량 계산

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
  
    last_sampling_time_dist = millis(); // 마지막 dist event 처리 시각 기록
  }
  
  if(event_servo) {
    event_servo = false; 
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
    duty_curr += duty_chg_per_interval;
    }
    else if(duty_curr > duty_target) {
      duty_curr -= duty_chg_per_interval;
      
    }
    
    
 
  //else {
    //duty_curr = duty_target;
    //duty_curr -= duty_chg_per_interval;
    //if(duty_curr < duty_target) duty_curr = duty_target;
  //}
   
    // update servo position
    myservo.writeMicroseconds(duty_curr);    
    last_sampling_time_servo = millis(); // 마지막 servo event 처리 시각 기록
  }
  
  
  if(event_serial) {
    event_serial = false; 
    Serial.print(duty_chg_per_interval );
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    last_sampling_time_serial = millis(); // 마지막 serial event 처리 시각 기록

  }

}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered(){
  if(dist_ema>204){
    return 50/20*(dist_ema-204)+350;
  }
  else if (dist_ema>185){
    return 50/19*(dist_ema-185)+300;
  }
  else if (dist_ema>177){
    return 50/8*(dist_ema-177)+250;
  }
  else if(dist_ema>153){
    return 50/31*(dist_ema-146)+200;
  }
  else if (dist_ema>98){
    return 50/48*(dist_ema-98)+150;
  }
  else {
    return 50/29*(dist_ema-69)+100;
  }
  }
