// WRITTEN      | 9/5/24 TO CONTROL MULTIPLE MOTORS WITH EMG
// LAST UPDATED | 10/19/24 RE-ADDED EMG CONTROL BECAUSE WHY WOULD YOU NEGATE THAT?

#include <math.h> // For the sin function
#include <ESP32Servo.h> // For the SERVO

//Ouput Constants
#define PWM 4  // Example pin, can be used for PWM
#define IN1 25  // Example pin, can be used for digital I/O
#define IN2 26  // Example pin, can be used for digital I/O
#define m1_encB 14  // Example pin, can be used for digital I/O and external interrupts
#define m1_encA 32  // Example pin, can be used for digital I/O and external interrupts
#define PWM_CHANNEL 4
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 4000
#define SERVO_PIN 18

//input constants
#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN 27
#define CAR_SIZE 50

//Position of the motors UPDATE WITH THE IDLE POSITIONS FOR THE HAND
int pos1 = 2;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;
int pos5 = 0;
int pos6 = 0;
int dir = 0;

int setPoint = 0;

//Controller variables
long prev_T  = 0;
float prev_err = 0;

//PID Gains (could also be placed inside the void loop)
float Kp = 6;
float Ki = .001;
float Kd = .4;

// Parameters for the sine wave
const float amplitude = 1000.0; // Maximum deviation from the midpoint
const float frequency = 1.2;  // Number of cycles per second
const float midpoint = 0.0;   // Center point of the sine wave

//Boxcar averager (for smooth EMG reading)
int bcsum = 0;
int boxcar[CAR_SIZE];
int bcn = 0;
int average_emg;
int counter = 0;
int counter_limit = 25;

//Arm state machine variables
bool engage_arm_clench = false;
float clench_threshold_value = 120; // set for each person

//Servo controls
Servo mini_servo;
int mini_servo_pos = 90; //90 is stop, 180 is full forward, 0 is full reverse

void setup() {
  pinMode(m1_encA, INPUT);
  pinMode(m1_encB, INPUT);

  pinMode(INPUT_PIN, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(PWM, OUTPUT);

  Serial.begin(115200);

  // Setup PWM
  ledcAttach(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWM, PWM_CHANNEL, PWM_RESOLUTION);

  attachInterrupt(digitalPinToInterrupt(m1_encA), incrementEncoder, RISING);

  //servo stuff
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  mini_servo.setPeriodHertz(50);
  mini_servo.attach(SERVO_PIN, 700, 2300); 
  mini_servo.write(90);
}

void loop() {
  //output stuffs
  long currentTime = micros();
  //int setPoint = (int) getSetPoint(currentTime);

  // EMG input
  float sensor_value = analogRead(INPUT_PIN);
  update_boxcar(sensor_value);

  if (average_emg > clench_threshold_value) {
    setPoint = 4000;
  }
  else{
    setPoint = 0;
  }

  motor(setPoint, pos1);

  delay(10);

}

void Motor_pwr(int dir, int PWM_val, int PWM_pin, int in1, int in2) {

  ledcWrite(PWM_CHANNEL, PWM_val); // Use ledcWrite for ESP32 PWM
  //Serial.println("dir, pwm_val");
  //Serial.println(dir);
  //Serial.println(PWM_val);
  
  if (dir == 1) { //forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    if(PWM_val > 300){
    mini_servo.write(180);
    }
    else{
      mini_servo.write(90);
    }
  }
  else if (dir == -1) { //back
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    if(PWM_val > 300){
    mini_servo.write(0);
    }
    else{
      mini_servo.write(90);
    }
  }
  else if (dir == 0) { //stall
    digitalWrite(in2, HIGH);
    digitalWrite(in1, HIGH);
    mini_servo.write(90);
  }
  //mini_servo.write(90);
}

float getSetPoint(long currentTime) {
  float timeInSeconds = ((float)currentTime) / 1.0e6;  // Convert microseconds to seconds
  float setPoint = midpoint + amplitude * sin(2.0 * M_PI * frequency * timeInSeconds);
  return setPoint;
}

void incrementEncoder() {
  pos1 = pos1 + dir;

}
void motor(int targ, int pos) {
  long current_T = micros();
  float delta_T = ((float)(current_T - prev_T)) / 1.0e6;
  prev_T = current_T;

  float err = targ - pos;
  //float err = 1000;
  float derivative = (err - prev_err) / delta_T;
  float integral = ((err + prev_err)/2) * delta_T;
  float output = Kp * err + Ki * integral + Kd * derivative;

  float pwr = fabs(output) * 2;
  if (pwr > 1000) {
    pwr = 1000;
  }

  dir = 1;
  
  if (output < targ) {
    dir = -1;
  }
  if(pwr < 300){ //if this produces unintended behavior, shoot me! -Alex
    dir = 0;
  }
  

  Motor_pwr(dir, pwr, PWM, IN1, IN2);

  prev_err = err;

}


float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732 * z1 - 0.36347401 * z2;
    output = 0.01856301 * x + 0.03712602 * z1 + 0.01856301 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795 * z1 - 0.39764934 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594 * z1 - 0.70744137 * z2;
    output = 1.00000000 * x + 2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112 * z1 - 0.74520226 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

void update_boxcar(float sensor_value) {
  //filter signal
  float filtered_in = abs(EMGFilter(sensor_value));
  //updating boxcar array
  bcsum -= boxcar[bcn];
  bcsum += filtered_in;
  boxcar[bcn] = filtered_in;
  bcn += 1;
  if (bcn >= CAR_SIZE)
    bcn = 0;
  average_emg = bcsum / CAR_SIZE;
}
