//2127
//27546
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
//#include <Wire.h>
#include "Adafruit_VL53L0X.h"

//_______TOF PARAMETERS___________________
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x13
#define LOX2_ADDRESS 0x16

// set the pins to shutdown
#define SHT_LOX1 8
#define SHT_LOX2 2

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

//_______IMU PARAMETERS__________________________
Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

double min_x, max_x, mid_x;
double min_y, max_y, mid_y;
double min_z, max_z, mid_z;

//Magnetometer Variables
const int period = 5000; 
bool mag_init = false;

int startMillis;


//Distance turn values
int minimum_side_dist = 999;
int minimum_front_dist = 999; //For now while testing

//NANO 
const int ledPin = LED_BUILTIN;

//______CONTROL INPUT PARAMETERS_______________________
int target_yaw = 10; //Same as box heading merge variable into one
double angle_error;
double rotation_integral = 0; 
int target_angle = 0;

//______CONTROL OUTPUT PARAMETERS______________________
int steering;
int forward;

int LeftMotorSpeed;
int RightMotorSpeed;

const int RightMotorForward = 6;
const int RightMotorBack = 5;

const int LeftMotorForward = 9;
const int LeftMotorBack = 10;

enum State{
    none,
    driving,
    turning,
    done
};

State state = turning;
  
void setup() {

  Serial.begin(115200);
  Serial.println("Starting");

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  //Interface setup
  pinMode(ledPin, OUTPUT);

  //_________________MOTOR SETUP_________________
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBack, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBack, OUTPUT);

  //setup_imu();
  setup_tof();

  startMillis = millis();
  Serial.println("Startup Completed.");
  //debug_imu();
}

void loop() {

  //_________ULTRASONIC LOOP FUNCTIONS________
  read_dual_sensors();
  log_tof_sensors();

  //_____________IMU LOOP FUNCTIONS___________
  //icm.getEvent(&accel, &gyro, &temp, &mag);
  //mag_bill();

  //_____________MOTOR LOOP FUNCTIONS_________
  //motor_test_forward();
  //motor_control();
  //motor_test();
  //delay(100);

  //____________CONTROL LOOP FUNCTIONS________
  //navigate_compass();
  //forward_drive();

  //align_from_left();
  //state = turning;
}

double Side_Distance_Reading;
double Front_Distance_Reading;

double Prev_Front_Distance_Reading = 0;
double Prev_Side_Distance_Reading = 0;

double Delta_Front_Distance_Reading = 0;
double Delta_Side_Distance_Reading = 0;

void setup_tof(){
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.print("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX1, INPUT);
  digitalWrite(SHT_LOX2, INPUT);

  Serial.print("Both in reset mode...(pins are low)");
  
  
  Serial.print("Starting...");
  setID();
}

void log_tof_sensors(){
  Serial.print("Front Distance: ");
  Serial.println(Front_Distance_Reading);

  Serial.print("Side Distance: ");
  Serial.println(Side_Distance_Reading);
}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(30);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(30);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  //digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.println();
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(" Range 1: "); Serial.print(measure1.RangeMilliMeter);
    Front_Distance_Reading = measure1.RangeMilliMeter;
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();

  // print sensor two reading
  if(measure2.RangeStatus != 4) {
    Serial.print(" Range 2: ");  Serial.print(measure2.RangeMilliMeter);
    Side_Distance_Reading = measure1.RangeMilliMeter;
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();

  Delta_Front_Distance_Reading = Front_Distance_Reading - Prev_Front_Distance_Reading;
  Prev_Front_Distance_Reading = Front_Distance_Reading;

  Delta_Side_Distance_Reading = Side_Distance_Reading - Prev_Side_Distance_Reading;
  Prev_Side_Distance_Reading = Side_Distance_Reading;
}


void setup_imu(){
 //_________________IMU SETUP___________________
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }

    // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  delay(3000);
}

float angle = 0;
float angle_offset = 0;
double prev_magnitude = 0;
void mag_bill(){
  icm.getEvent(&accel, &gyro, &temp, &mag);
  
  /*mag->getEvent(&event);
  float x = event.magnetic.x;
  float y = event.magnetic.y;
  float z = event.magnetic.z;*/
  
  float x = mag.magnetic.x;
  float y = mag.magnetic.y;
  float z = mag.magnetic.z;
  
  //Serial.print("Mag: (");
  //Serial.print(x); Serial.print(", ");
  //Serial.print(y); Serial.print(", ");
  //Serial.print(z); Serial.print(") ");
  //Serial.print(sqrt((x*x)+(y*y)+(z*z)));

  min_x = min(min_x, x);
  min_y = min(min_y, y);
  min_z = min(min_z, z);

  max_x = max(max_x, x);
  max_y = max(max_y, y);
  max_z = max(max_z, z);

  mid_x = (max_x + min_x) / 2;
  mid_y = (max_y + min_y) / 2;
  mid_z = (max_z + min_z) / 2;

  // adjusted values
  float adj_x = x-mid_x;
  float adj_y = y-mid_y;
  float adj_z = z-mid_z;
  
  float magnitude = sqrt((adj_x*adj_x)+(adj_y*adj_y)+(adj_z*adj_z));
  /*Serial.print("Magnitude: (");
  Serial.print(magnitude); Serial.print(") ");
  
  Serial.print(" Hard offset: (");
  Serial.print(mid_x); Serial.print(", ");
  Serial.print(mid_y); Serial.print(", ");
  Serial.print(mid_z); Serial.print(")");  

  Serial.print(" Field: (");
  Serial.print((max_x - min_x)/2); Serial.print(", ");
  Serial.print((max_y - min_y)/2); Serial.print(", ");
  Serial.print((max_z - min_z)/2); Serial.println(")"); 

  Serial.print(" Readings ADJ: (");
  Serial.print(adj_x); Serial.print(", ");
  Serial.print(adj_y); Serial.print(", ");
  Serial.print(adj_z); Serial.println(")"); */

  angle = atan2(y-mid_y, x-mid_x);
  angle = angle - angle_offset;
  //Serial.print(" Angle: (");
  //Serial.print(angle*57.2958); Serial.print(")");

  if(magnitude == prev_magnitude){
    setup_imu();
  }
  magnitude = prev_magnitude;
  //delay(1000); 

  angle_error = angle - target_angle;
    
  if(angle_error > 180)
    angle_error -= 360;
  if(angle_error < -180)
    angle_error += 360;

}

void pulse_led(){
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
}

//Used for the initial setup of the robot
bool both_tof_covered(){
  return Front_Distance_Reading < 1 && Side_Distance_Reading < 1;
}

float headings [6] =       {270, 0, 90, 180, 270, 0,};
float front_distance_drive [6] = {20,   20,  20,   40,   40, 40,};
float front_distance_turn [6] = {20,   20,  20,   40,   40, 40,};
float side_distance [6] =  {20,   20,  20,   20,   40, 40,};
int navigation_index = 0;
bool magnetomer_setup = false;
bool heading_offset = false;

void navigate_compass(){ 
  if(!magnetomer_setup){
    if (both_tof_covered()){
      magnetomer_setup = true;
      Serial.println("Mag setup");
      pulse_led();
    }
    return;
  }

  if(!heading_offset){
    if (both_tof_covered()){
      angle_offset = angle;
      heading_offset = true;
      Serial.println("Heading offset");
      pulse_led();
    }
    return;
  }

  //rotation_drive();
  forward_drive();

  //Check rotation error calculation
  /*
  if (abs(angle_error)*57.2958 > 15 && state == turning) {
    rotation_drive();
    state = turning;
  }
  else{
    steering = 0;
    rotation_integral = 0;
    state = driving;
    delay(1000);
  }

  int allowable_forward_error = 2;
  if(state == driving){
    forward_drive();
    state = driving;
    if (Front_Distance_Reading > front_distance_drive[navigation_index] - allowable_forward_error &&
        Front_Distance_Reading < front_distance_drive[navigation_index] + allowable_forward_error){
      state = turning;
      forward = 0;
      navigation_index++;
      delay(1000);
    }
  }

  motor_control();*/
}

void motor_control(){
  RightMotorSpeed = steering + forward;
  LeftMotorSpeed = -steering + forward;

  double max_motor_speed = 160;

  RightMotorSpeed = constrain(RightMotorSpeed, -max_motor_speed, max_motor_speed);
  LeftMotorSpeed = constrain(LeftMotorSpeed, -max_motor_speed, max_motor_speed);

  if(RightMotorSpeed <= 0){
    analogWrite(RightMotorBack, -RightMotorSpeed);
    analogWrite(RightMotorForward,0);
  }
  if(RightMotorSpeed >= 0){
    analogWrite(RightMotorBack, 0);
    analogWrite(RightMotorForward, RightMotorSpeed);
  }

  if(LeftMotorSpeed <= 0){
    analogWrite(LeftMotorBack, -LeftMotorSpeed);
    analogWrite(LeftMotorForward,0);
  }
  if(LeftMotorSpeed >= 0){
    analogWrite(LeftMotorBack, 0);
    analogWrite(LeftMotorForward, LeftMotorSpeed);
  }

  Serial.println("");
  Serial.print(" Steering: ");  Serial.print(steering);
  Serial.print(" Forward: ");  Serial.print(forward);
  Serial.print(" Side Distance: ");  Serial.print(Side_Distance_Reading);
  Serial.print(" Forward Distance: ");  Serial.print(Front_Distance_Reading);
  Serial.print(" State: ");  Serial.print(state);
  Serial.print(" Angle: ");  Serial.print(angle*57.2958);
  Serial.print(" Angle Error: ");  Serial.print(angle_error*57.2958);
  Serial.print(" Angle Offset: ");  Serial.print(angle_offset*57.2958);
}

double rot_kp = -0.2;
double rot_ki = 0;
double rot_kd = -2;
double turning_input = 0;
double prev_error = 0;
void rotation_drive(){
    if (abs(angle_error) < 5) {
      steering = 0;
      return;
    }
    
    Serial.print("Angle error: (");
    Serial.print(angle_error); Serial.print(") ");
    
    double error_derivative = angle_error - prev_error;
    rotation_integral += angle_error;

    //TODO: Define this number in volts to account for changing battery voltage.
    steering = angle_error * rot_kp + error_derivative * rot_kd; // + rotation_integral * rot_ki;
    steering += steering/abs(steering) * 120;
    prev_error = angle_error;
}

float forward_intergral = 0;
float forward_error = 999;
float forward_kp = 30;
float forward_ki = 0.06f;
float forward_kd = 10;
//Balance heading heading constant with maximising forward motion and keeping wall distances within parameters.
void forward_drive(){
  forward_error = Front_Distance_Reading - front_distance_drive[navigation_index];
  double side_error = Side_Distance_Reading - side_distance[navigation_index];

  //Generate motion guidance
  if (forward_error > 4){
    //forward = forward_error * forward_kp * 30;
    forward = 200;
    steering = side_error * 30;
    //steering = angle_error * rot_kp;
  }     //Terminal Guidance
  /*else if (forward_error < 4){
    forward = (forward_error - 2) * forward_kp*30;
    steering = angle_error * rot_kp/10;
  }*/

  Serial.println();
  Serial.println(forward);
  Serial.println(side_error);
  Serial.println(side_error * 30);
  //Serial.println();

  double total_control = steering + forward;
  if(total_control > 255){
    steering /= total_control*255;
    forward /= total_control*255;
  }

  Serial.println();
  Serial.println(forward_error);
  Serial.println();
}

double drive_rot_kp = -0.2;
float allowable_angle_error = 2.5;
int allowable_front_error = 20;
int allowable_side_error = 20;

void adjust_drive_with_heading_bill() {  
  forward_error = Front_Distance_Reading - front_distance_drive[navigation_index];
  double cur_side_dist = side_distance[navigation_index];
  double side_error = Side_Distance_Reading - side_distance[navigation_index];

  /* using angle error with distance
    --- no side error ---
    measured error <= theoretical value within 4 cm
    abs(side_error) <= side_distance - side_distance[navigation_index]/cos(angle_error) + 40;

    old: abs(side_error) <= 3*(navigation_index+1)
  */
  
  if (abs(forward_error) > allowable_front_error) {
    forward = 200;

    /* 
      // slow/stop robot if angle error is too high 
      if (abs(angle_error*57.2958) > 10) {
        forward = 100;
      }
    */
    
    // the thing is on course and moving forward
    if (abs(angle_error*57.2958) <= allowable_angle_error && abs(side_error) <= abs(cur_side_dist - cur_side_dist/cos(angle_error)) + allowable_side_error) {
      steering = 0;
    }
    // angle error present, no side error
    else if (abs(angle_error*57.2958) > allowable_angle_error && abs(side_error) <= abs(cur_side_dist - cur_side_dist/cos(angle_error)) + allowable_side_error) {
      steering = angle_error * drive_rot_kp; // can tune the rot_kp here
    }
    // side error present, no angle error
    else if (abs(angle_error*57.2958) <= allowable_angle_error && abs(side_error) > abs(cur_side_dist - cur_side_dist/cos(angle_error)) + allowable_side_error) {
      steering = side_error * drive_rot_kp; // can tune the rot_kp here
    }
    // angle and side error present
    else {
      steering = angle_error * drive_rot_kp;
    }
  }
  else {
    // stop forward motion
    forward = 0;
    // state = 
  }
}

void motor_test_forward(){
  double speed = 255;
  analogWrite(RightMotorForward, speed);
  analogWrite(RightMotorBack, 0);
  analogWrite(LeftMotorForward, speed);
  analogWrite(LeftMotorBack, 0);
  delay(10000);
   Serial.println("Stop all");
  analogWrite(RightMotorForward, 0);
  analogWrite(RightMotorBack, 0);
  analogWrite(LeftMotorForward, 0);
  analogWrite(LeftMotorBack, 0);
  delay(2000000);
}

void motor_test(){
  Serial.println("Backward both");
  analogWrite(RightMotorForward, 0);
  analogWrite(RightMotorBack, 255);
  analogWrite(LeftMotorForward, 0);
  analogWrite(LeftMotorBack, 255);
  delay(2000);
  Serial.println("Forward both");
  analogWrite(RightMotorForward, 255);
  analogWrite(RightMotorBack, 0);
  analogWrite(LeftMotorForward, 255);
  analogWrite(LeftMotorBack, 0);
  delay(2000);
  Serial.println("Right turn");
  analogWrite(RightMotorForward, 0);
  analogWrite(RightMotorBack, 255);
  analogWrite(LeftMotorForward, 255);
  analogWrite(LeftMotorBack, 0);
  delay(2000);
  Serial.println("Left turn");
  analogWrite(RightMotorForward, 255);
  analogWrite(RightMotorBack, 0);
  analogWrite(LeftMotorForward, 0);
  analogWrite(LeftMotorBack, 255);
  delay(2000);
  Serial.println("Stop all");
  analogWrite(RightMotorForward, 0);
  analogWrite(RightMotorBack, 0);
  analogWrite(LeftMotorForward, 0);
  analogWrite(LeftMotorBack, 0);
  delay(2000000);
}

void setup_drive(){
  steering = 0;
  //rotation_intergral = 0;
  state = driving;
}


double turn_increment_timer = 3; //Hardcoded timer so the robot turns past the wall ideally
double turn_time;
void setup_turn(){
  turn_time = millis();
  state = turning;
  forward = 0;
}
