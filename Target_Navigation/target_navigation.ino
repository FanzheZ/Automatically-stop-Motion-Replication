// ROS Test 
// Tests reading and publishing of GPS and IMU

#if ARDUINO>=100
  #include <Arduino.h>  // Arduino 1.0
#else
  #include <WProgram.h>  // Arduino 0022
#endif

// ROS includes
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>

// Arduino libraries

#include <Servo.h> // motor
#include "NewPing.h" // sensor
#include <Wire.h>
#include <MsTimer2.h>
#include <EEPROM.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BNO055.h>

#define IMU_PUBLISH_RATE 15 //hz
#define GPS_PUBLISH_RATE 5 //hz
#define SENSOR_PUBLISH_RATE 1

// ROS message handlers and publishers
ros::NodeHandle nh;
std_msgs::Float32 Distancef;
//std_msgs::Float32 Distanceb;
//std_msgs::Float32 Distancel;
std_msgs::Float32 Distancer;
sensor_msgs::NavSatFix navSat_msg;
std_msgs::Float32 latitude;
std_msgs::Float32 longitude;
std_msgs::Float32 x_direction;

ros::Publisher pub_rangef("frontsensor",&Distancef);
//ros::Publisher pub_rangeb("backsensor",&Distanceb);
//ros::Publisher pub_rangel("leftsensor",&Distancel);
ros::Publisher pub_ranger("rightsensor",&Distancer);
ros::Publisher xDirection("x_angle",&x_direction);
ros::Publisher gpsPub("gps", &navSat_msg);
Adafruit_BNO055 bno = Adafruit_BNO055(55);


// GPS definitions
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing to the Serial console
// Set to 'true' to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();
uint32_t publish_imu_time = 0;
uint32_t publish_gps_time = 0;
uint32_t publish_sensor_time=0;
uint32_t publish_turning_time=0;

// define target location
float target_latitude=37.2314300537;
float target_longitude=-80.4251861572;
#define PI atan2(0, -1)
float gpslati,gpslong;
float target_angle,heading,heading_difference;
bool cal_angle=true;

// motor definitions
Servo lfMotor;
Servo rfMotor;
Servo lbMotor;
Servo rbMotor;

#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define LEFT2 'N'
#define RIGHTBACK 'V'
#define STOP 'S'
#define WANDER 'W'

static float WIDTH = 0.273;   // meters
static float RADIUS = 0.120;  // meters

static unsigned char MOTORS_PWM_MIN = 40;
static unsigned char MOTORS_PWM_ZERO = 94;
static unsigned char MOTORS_PWM_MAX = 150;

static char OMEGA_MAX = (2*1 + 1*WIDTH)/(2*RADIUS);


// Sensor definitions
// define all the pins with sensors
#define trigPinF 24
#define echoPinF 25
#define trigPinB 10
#define echoPinB 11
#define trigPinL 22
#define echoPinL 23
#define trigPinR 34
#define echoPinR 35
#define MAX_distance 400


NewPing sonarf(trigPinF, echoPinF, MAX_distance);
NewPing sonarb(trigPinB, echoPinB, MAX_distance);
NewPing sonarl(trigPinL, echoPinL, MAX_distance);
NewPing sonarr(trigPinR, echoPinR, MAX_distance);
void sonarResult();

float durationf, distancef,durationb, distanceb,durationl, distancel,durationr, distancer;
float tmp;
float soundsp=340;  // Stores calculated speed of sound in M/S
float soundcm=340/10000;  // Stores calculated speed of sound in cm/ms
int iterations = 5;


/*
*
* Functions for Motor
*
 */
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
  lfMotor.write(lf);
  rfMotor.write(rf);
  lbMotor.write(lb);
  rbMotor.write(rb);
}


void parseInput(char cmd){ // read the input and react to the input
  switch(cmd) {

      case FORWARD:
      cmdvelCb(0.15,0);
      break;

      case LEFT:
      cmdvelCb(0.2,-2);
      break;

      case RIGHT:
      cmdvelCb(0.2,2);
      break;

      case LEFT2: 
      cmdvelCb(0.5,-1.57);  
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
  }
}


void input_char(const std_msgs::Char& input_cmd){ // read input from the keyboard
  bool set=false;
  char cmd;
  
  cmd=input_cmd.data;

  parseInput(cmd);
    
}
ros::Subscriber<std_msgs::Char> sub("input" , input_char);

/*
 * All about sensor
 */

void readSonars() { // read sensor 
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

void navigate_with_sonar(){ // react to the sensor data
      readSonars();
  
      // when front detect object, turn left
      if ((distancef<=50 && distancef>0)){
         cmdvelCb(0.5,-1.57);
      }
     else if (distancer<=50&& distancer>0)
     {
        // when front detect object, stop.
        if ((distancef<=50 && distancef>0)){
          parseInput(STOP);
        }

        else
        {
          //  Serial.print("L");
            parseInput(LEFT);
            //delay(4200);
            //parseInput(FORWARD);
        }
     }
 }


bool at_goal(){
    if(gpslati>=target_latitude-0.00005 && gpslati<=target_latitude+0.00005
          && gpslong>=target_longitude-0.00005 && gpslong<=target_longitude+0.00005)
          return true;
    else
          return false;
      
}

float cal_goal_angle(float current_lati, float current_long,float target_latitude,float target_longitude){

       float X=cos(target_latitude)*sin(target_longitude-current_long);
       float Y=cos(current_lati)*sin(target_latitude)-sin(current_lati)*cos(target_latitude)*cos(target_longitude-current_long);
       float bearing=atan2(X,Y); //in radian
       target_angle=bearing*180/PI;

       if(bearing>=0 && bearing<=180){
          target_angle=target_angle;
       }
       else if(bearing<0 && bearing>=-180){
          target_angle=target_angle+360;
       }
       else if(bearing==360){
          target_angle=0;
       }

      
        
        return target_angle;
          
}


float check_angle_diff(float heading, float target_angle){
      heading_difference= target_angle-heading;
      return heading_difference; // positive heading_difference means the target is at right
}


void setup() {
 
  lfMotor.attach(2); 
  rfMotor.attach(3); 
  lbMotor.attach(4);
  rbMotor.attach(5);
  
  // initialize ROS node
  nh.initNode();
  nh.getHardware()->setBaud(57600);

  //topic to publish/subscribe
  nh.subscribe(sub);
  nh.advertise(pub_rangef);
  nh.advertise(pub_ranger);
  nh.advertise(gpsPub);
  nh.advertise(xDirection); 
  
 /* Setup imu*/
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  int eeAddress = 0;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      nh.loginfo("No Calibration Data for this sensor exists in EEPROM");
      //Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      nh.loginfo("Found Calibration for this sensor in EEPROM.");
      //Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      nh.loginfo("Restoring Calibration data to the BNO055...");
      //Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      nh.loginfo("Calibration data loaded into BNO055");
      //Serial.println("\n\nCalibration data loaded into BNO055");
  }
  
  delay(1000);

  bno.setMode(bno.OPERATION_MODE_COMPASS);
   
  bno.setExtCrystalUse(true);
      
  /*setup GPS*/
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
  GPS.begin(9600);
  // uncomment this line to turn on RMC 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but
  // either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  while (!nh.connected()) {
    nh.spinOnce();
  }
}

unsigned long previousMillis = 0; //Will store time that the sensor last ran

const long interval = 1000;      //interval at which the sensor will start 

bool following_obstacle=false;

bool front,right=false;

void loop() {

        nh.spinOnce();
       
        unsigned long currentMillis = millis(); //currentMillis is the time that the arduino has been running in milliseconds
    
        if ((millis() - publish_imu_time) >= (1000 / IMU_PUBLISH_RATE))
        {  //should be a 1 second delay without the delay function
            sensors_event_t event; 
            bno.getEvent(&event);
          
            x_direction.data=event.orientation.x;
            xDirection.publish(&x_direction);
            heading=event.orientation.x;
          
            publish_imu_time = millis();
        }


        char c = GPS.read();
        
          if (GPS.newNMEAreceived()) {
            if (!GPS.parse(GPS.lastNMEA())) 
              return; 
          }
        
          //this block publishes velocity based on defined rate
          if ((millis() - publish_gps_time) >= (1000 / GPS_PUBLISH_RATE))
          {
              float lat=GPS.latitude/100;
              double fractpart,intpart;
              fractpart = modf (lat, &intpart);
              fractpart=fractpart/0.6;
              gpslati=intpart+fractpart;
          
              float lon=GPS.longitude/100;
              double fractpart2,intpart2;
              fractpart2 = modf (lon, &intpart2);
              fractpart2=fractpart2/0.6;
              gpslong=-(intpart2+fractpart2);
          
              navSat_msg.latitude = gpslati;
              navSat_msg.longitude = gpslong;
              navSat_msg.altitude = GPS.altitude;
              gpsPub.publish(&navSat_msg);
            
              publish_gps_time = millis();
      
              if(GPS.fix!=0){
                     nh.loginfo("GPS FIX!!!!!!!!!");
                     
                     target_angle=cal_goal_angle(gpslati,gpslong,target_latitude,target_longitude);
                     nh.loginfo("calculated the angle");

                     heading_difference=target_angle-heading;
                     nh.loginfo("calculated angle difference");


                     if(at_goal()){
                          nh.loginfo("At goal.");
                          parseInput(STOP);
                          while(1){
                          }
                     }
                     else{
                                 nh.loginfo("turning toward target");

                                 if((heading_difference>=-5 && heading_difference<=5) || (heading_difference>=355 && heading_difference<=360) || (heading_difference>=-360 && heading_difference<=-355)){ 
                                     nh.loginfo("GOING TO TARGET: FORWARD");
                                     parseInput(FORWARD);
                                 }
                                 else if(heading_difference>5 && heading_difference<=180){ 
                                     nh.loginfo("GOING TO TARGET: turning RIGHT");
                                     parseInput(RIGHT);
                                 }
                                 else if(heading_difference > 180 && heading_difference<355){
                                     nh.loginfo("GOING TO TARGET: turning LEFT");
                                     parseInput(LEFT);
                                 }
                                 else if(heading_difference<-5 && heading_difference>=-180){ 
                                     nh.loginfo("GOING TO TARGET: turning LEFT");
                                     parseInput(LEFT);
                                 }
                                 else if(heading_difference<-180 && heading_difference>-355){
                                     nh.loginfo("GOING TO TARGET: turning RIGHT");
                                     parseInput(RIGHT);
                                 }
                        }
              }
              
            }
 
 }// end loop



   

 
