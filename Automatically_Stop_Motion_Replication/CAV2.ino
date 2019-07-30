#include <Servo.h>
#include "NewPing.h"

// define all the pins with sensors
#define trigPinF 12
#define echoPinF 13
#define trigPinB 10
#define echoPinB 11
#define trigPinL 8
#define echoPinL 9
#define trigPinR 6
#define echoPinR 7
#define MAX_distance 400


#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define LEFTBACK 'N'
#define RIGHTBACK 'V'
#define STOP 'S'
/*
#define FORWARD 'f'
#define BACKWARD 'b'
#define LEFT 'l'
#define RIGHT 'r'
#define LEFTBACK 'n'
#define RIGHTBACK 'v'
#define STOP 's'*/


Servo LfMotor;
Servo RfMotor;
Servo LbMotor;
Servo RbMotor;

NewPing sonarf(trigPinF, echoPinF, MAX_distance);
NewPing sonarb(trigPinB, echoPinB, MAX_distance);
NewPing sonarl(trigPinL, echoPinL, MAX_distance);
NewPing sonarr(trigPinR, echoPinR, MAX_distance);

//definition about sensors
float durationf, distancef,durationb, distanceb,durationl, distancel,durationr, distancer;
float soundsp=340;  // Stores calculated speed of sound in M/S
float soundcm=340/10000;  // Stores calculated speed of sound in cm/ms
int iterations = 5;

//definition about CAV
static float WIDTH = 0.273;   // meters
static float RADIUS = 0.120;  // meters

static unsigned char MOTORS_PWM_MIN = 40;
static unsigned char MOTORS_PWM_ZERO = 94;
static unsigned char MOTORS_PWM_MAX = 150;

static char OMEGA_MAX = (2*1 + 1*WIDTH)/(2*RADIUS);

unsigned char mapToPWM(float v) {
  unsigned char pwm;
  
  if(v >= 0) {
    v = min(v,OMEGA_MAX)/OMEGA_MAX;
    pwm = (unsigned char)(v*(MOTORS_PWM_MAX-MOTORS_PWM_ZERO) + MOTORS_PWM_ZERO);
  } else {
    v = max(v,-OMEGA_MAX)/OMEGA_MAX;
    pwm = (unsigned char)(MOTORS_PWM_ZERO + v*(MOTORS_PWM_ZERO-MOTORS_PWM_MIN));
  }
  return pwm;
}


void cmdvelCb(double linear, double angular){
  
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  
  // Convert Twist into differential-drive 
  float wl, wr;
  wl = (2*linear + angular*WIDTH)/(2*RADIUS);
  wr = (2*linear - angular*WIDTH)/(2*RADIUS);
  
  setMotors(mapToPWM(wl),mapToPWM(wr),mapToPWM(wl),mapToPWM(wr));
}


void setMotors(unsigned char lf, unsigned char rf, unsigned char lb, unsigned char rb) {
  LfMotor.write(lf);
  RfMotor.write(rf);
  LbMotor.write(lb);
  RbMotor.write(rb);
}
  
void parseInput(char cmd){
  
  switch(cmd) {
    
    case FORWARD:
      cmdvelCb(0.2,0);
      break;


    case LEFT:
      setMotors(100,MOTORS_PWM_MAX,95,133);
      break;

    case RIGHT:
      setMotors(MOTORS_PWM_MAX,100,133,95); 
      break;

    case LEFTBACK: // left back
      cmdvelCb(-0.5,1.57);  
      break;

    case RIGHTBACK: //right back
      cmdvelCb(-0.5,-1.57);  
      break;

      case BACKWARD:
      cmdvelCb(-0.2,0);
      break;
      
      case STOP:
      cmdvelCb(0,0);
      break;

      case 'A': //square
      cmdvelCb(1,0);
      delay(1300); 
      setMotors(100,MOTORS_PWM_MAX,MOTORS_PWM_ZERO,130);
      delay(1165);
      cmdvelCb(1,0);
      delay(1300);
      setMotors(100,MOTORS_PWM_MAX,MOTORS_PWM_ZERO,130);
      delay(1165);
      cmdvelCb(1,0);
      delay(1300); 
      setMotors(100,MOTORS_PWM_MAX,MOTORS_PWM_ZERO,130);
      delay(1165);
      cmdvelCb(1,0);
      delay(1300); 
      setMotors(100,MOTORS_PWM_MAX,MOTORS_PWM_ZERO,130);
      delay(1165);
      cmdvelCb(1,0);
      delay(1000); 
      cmdvelCb(0,0);
      break;
      
      case 'C': //square
      cmdvelCb(1,0);
      delay(1300); 
      setMotors(100,MOTORS_PWM_MAX,95,133);
      delay(1170);
      cmdvelCb(1,0);
      delay(1300);
      setMotors(100,MOTORS_PWM_MAX,95,133);
      delay(1170);
      cmdvelCb(1,0);
      delay(1300); 
      setMotors(100,MOTORS_PWM_MAX,95,133);
      delay(1170);
      cmdvelCb(1,0);
      delay(1300); 
      setMotors(100,MOTORS_PWM_MAX,95,133);
      delay(1170);
      cmdvelCb(1,0);
      delay(1000); 
      cmdvelCb(0,0);
      break;

      case 'Z': // square
      cmdvelCb(1,0);
      delay(1300); 
      setMotors(MOTORS_PWM_MAX,100,133,95);
      delay(1170);
      cmdvelCb(1,0);
      delay(1300);
      setMotors(MOTORS_PWM_MAX,100,133,95);
      delay(1170);
      cmdvelCb(1,0);
      delay(1300); 
      setMotors(MOTORS_PWM_MAX,100,133,95);
      delay(1170);
      cmdvelCb(1,0);
      delay(1300); 
      setMotors(MOTORS_PWM_MAX,100,133,95);
      delay(1170);
      cmdvelCb(1,0);
      delay(1000); 
      cmdvelCb(0,0);
      break;
  }
}

void readSonars() {
  // result from sensors
  soundsp = 340;
  soundcm = soundsp / 10000;

  durationf = sonarf.ping_median(iterations);
  durationb = sonarb.ping_median(iterations);
  durationl = sonarl.ping_median(iterations);
  durationr = sonarr.ping_median(iterations);
  
  distancef = (durationf / 2) * soundcm;
  distanceb = (durationb / 2) * soundcm;
  distancel = (durationl / 2) * soundcm;
  distancer = (durationr / 2) * soundcm;  
}


void setup () {
  
  LfMotor.attach(2); 
  RfMotor.attach(3); 
  LbMotor.attach(4);
  RbMotor.attach(5);

  Serial.begin(9600);

}


void loop() {  
  
  if (Serial.available()) {
		parseInput((char)Serial.read());
    delay(1000);
  }
}
