#include <PID_v1.h>

// L298N Motor Driver Connection PINs
// Left Motor (Motor A)
#define LEFT_MOTOR_ENA 11   // PWM speed control for left motor
#define LEFT_MOTOR_IN1 10   // Direction pin 1 for left motor
#define LEFT_MOTOR_IN2 6    // Direction pin 2 for left motor

// Right Motor (Motor B)  
#define RIGHT_MOTOR_ENB 9   // PWM speed control for right motor
#define RIGHT_MOTOR_IN3 12  // Direction pin 1 for right motor
#define RIGHT_MOTOR_IN4 5   // Direction pin 2 for right motor

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 3  // Interrupt
#define right_encoder_phaseB A5  
#define left_encoder_phaseA 2   // Interrupt
#define left_encoder_phaseB A4

// Encoders
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String left_wheel_sign = "p";   // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0;     // rad/s
double left_wheel_cmd_vel = 0.0;      // rad/s
// Input - Measurement
double right_wheel_meas_vel = 0.0;    // rad/s
double left_wheel_meas_vel = 0.0;     // rad/s
// Output - Command
double right_wheel_cmd = 0.0;         // 0-255
double left_wheel_cmd = 0.0;          // 0-255
// Tuning (Your original PID values)
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;
// Controller
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  // Initialize L298N Motor Driver Pins
  pinMode(LEFT_MOTOR_ENA, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  
  pinMode(RIGHT_MOTOR_ENB, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);

  // Set initial motor direction to forward
  setLeftMotorDirection(true);   // true = forward
  setRightMotorDirection(true);  // true = forward
  
  // Initialize PID controllers
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  
  // Serial communication for ROS2
  Serial.begin(115200);

  // Initialize encoders
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available()) {
    char chr = Serial.read();
    
    // Right Wheel Motor
    if(chr == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Motor
    else if(chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if(chr == 'p') {
      if(is_right_wheel_cmd && !is_right_wheel_forward) {
        setRightMotorDirection(true);
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward) {
        setLeftMotorDirection(true);
        is_left_wheel_forward = true;
      }
    }
    // Negative direction
    else if(chr == 'n') {
      if(is_right_wheel_cmd && is_right_wheel_forward) {
        setRightMotorDirection(false);
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward) {
        setLeftMotorDirection(false);
        is_left_wheel_forward = false;
      }
    }
    // Separator
    else if(chr == ',') {
      if(is_right_wheel_cmd) {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd) {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Command Value
    else {
      if(value_idx < 5) {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder processing
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval) {
    // Convert to rad/s using author's approach but with your encoder specs
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0/255.0)) * 0.10472;
    left_wheel_meas_vel = (10 * left_encoder_counter * (60.0/255.0)) * 0.10472;
    
    // Compute PID output
    rightMotor.Compute();
    leftMotor.Compute();

    // Stop motors if command velocity is zero
    if(right_wheel_cmd_vel == 0.0) {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_cmd_vel == 0.0) {
      left_wheel_cmd = 0.0;
    }

    // Send encoder readings back to ROS2
    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    
    // Reset counters and update time
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    // Apply motor commands using L298N PWM control
    analogWrite(LEFT_MOTOR_ENA, left_wheel_cmd);    // Left motor speed
    analogWrite(RIGHT_MOTOR_ENB, right_wheel_cmd);  // Right motor speed
  }
}

// Function to set left motor direction
void setLeftMotorDirection(bool forward) {
  if(forward) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
  }
}

// Function to set right motor direction  
void setRightMotorDirection(bool forward) {
  if(forward) {
    digitalWrite(RIGHT_MOTOR_IN3, HIGH);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
  } else {
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, HIGH);
  }
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback() {
  // Determine direction based on phase B
  if(digitalRead(right_encoder_phaseB) == HIGH) {
    right_wheel_sign = "p";
  } else {
    right_wheel_sign = "n";
  }
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback() {
  // Determine direction based on phase B (matching author's pattern)
  if(digitalRead(left_encoder_phaseB) == HIGH) {
    left_wheel_sign = "n";
  } else {
    left_wheel_sign = "p";
  }
  left_encoder_counter++;
}