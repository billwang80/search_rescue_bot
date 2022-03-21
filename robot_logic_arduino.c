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

void ISR_countA() {
  counter_A++;
}

void ISR_countB() {
  counter_B++;
}

// convert centimeter to steps
int cm_to_steps(float cm) {
  int res;

  float circumference = (wheel_diameter * 3.1415) / 10;
  float cm_step = circumference / step_count;

  float float_res = cm / cm_step;
  res = (int) float_res;

  return res
}


void setup() {

}


