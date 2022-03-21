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

void move_forward(int steps, int mspeed) {
  counter_A = 0;
  counter_B = 0;

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  while (steps > counter_A && steps > counter_B) {
    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }

    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // stop once finished
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;
  counter_B = 0;
}

void move_reverse(int steps, int mspeed) {
  counter_A = 0;
  counter_B = 0;

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  while (steps > counter_A && steps > counter_B) {
    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }

    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // stop once finished
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;
  counter_B = 0;
}

void setup() {

}


