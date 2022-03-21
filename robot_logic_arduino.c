// motor interrupt pins
const byte MOTOR_A = 3; 
const byte MOTOR_B = 2;

// steps in disk
const float step_count = 10.0;

// wheel diameter
const float wheel_diameter = 100.0;

volatile int counter_A = 0;
volatile int counter_B = 0;

// motor A
int enA = 10;
int in1 = 9;
int in2 = 8;

// motor B
int enB = 5;
int in3 = 7;
int in4 = 8; 

// ---------- ISRs ----------




