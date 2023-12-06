#include <Wire.h>
#include <ros.h>
#include <slam_itbdelabo/HardwareCommand.h>
#include <Servo.h>


#include "Motor.h"
#include "Encoder.h"
#include "LPF.h"
#include "pidIr.h"

// For Debugging, uncomment one of these
//#define RECEIVER_RAW
//#define MOTOR_ANGLE_PULSE
//#define MOTOR_ANGLE_DEGREE
//#define MOTOR_ANGLE_RADIAN
//#define MOTOR_ANGLE_REVOLUTION
//#define MOTOR_SPEED_PPS
//#define MOTOR_SPEED_DPS
//#define MOTOR_SPEED_RPS
#define MOTOR_SPEED_RPM
#define TARGET_RPM
//#define PWM_RESPONSE
//#define VEHICLE_POSITION
//#define VEHICLE_SPEED
//#define EKF_DATA


// Receiver PIN
#define NUM_CH 8
#define PIN_CH_1 A8
#define PIN_CH_2 A9
#define PIN_CH_3 A10
#define PIN_CH_4 A11
#define PIN_CH_5 A12
#define PIN_CH_6 A13
#define PIN_CH_7 A14
#define PIN_CH_8 A15

// Motor PIN
#define RIGHT_MOTOR_REN_PIN 4
#define RIGHT_MOTOR_LEN_PIN 5
#define RIGHT_MOTOR_PWM_PIN 9

#define LEFT_MOTOR_REN_PIN 6
#define LEFT_MOTOR_LEN_PIN 7
#define LEFT_MOTOR_PWM_PIN 8

// Encoder PIN
#define RIGHT_ENC_PIN_A 50
#define RIGHT_ENC_PIN_B 51

#define LEFT_ENC_PIN_A 11
#define LEFT_ENC_PIN_B 10

// LED PIN
#define RED_LED  30
#define BLUE_LED 31

// SERVO
#define CAM_SERVO 2
#define MAX_SERVO_POS 175
#define MIN_SERVO_POS 125
#define INCREMENT_POS 10

// Constants
#define LOOP_TIME 10                // in milliseconds
#define PERIOD_TIME 2*pow(10,6)     // in microseconds
#define RECEIVER_LPF_CUT_OFF_FREQ 1 // in Hertz (Hz)
#define ENC_LPF_CUT_OFF_FREQ 3      // in Hertz (Hz)
#define PWM_THRESHOLD 150           // in microseconds of receiver signal
#define MAX_RPM_MOVE 180             // in RPM for longitudinal movement
#define MAX_RPM_TURN 70  // in RPM for rotational movement
#define WHEEL_RADIUS 5.0            // in cm
#define WHEEL_DISTANCE 33.0         // in cm
#define DISTANCE 200                // in cm
#define MAX_PWM 250                 // saturation PWM for action control (0-255)
#define ARMED 0x00                  // armed condition
#define DISARMED 0x01               // disarmed condition
#define HMC5983_ADDRESS 0x1E        // magnetometer I2C address

#define KP_RIGHT_MOTOR 1.0
#define KI_RIGHT_MOTOR 0.1
#define KD_RIGHT_MOTOR 10.0

#define KP_LEFT_MOTOR 1.0
#define KI_LEFT_MOTOR 0.1
#define KD_LEFT_MOTOR 10.0

Motor RightMotor(RIGHT_MOTOR_REN_PIN, RIGHT_MOTOR_LEN_PIN, RIGHT_MOTOR_PWM_PIN);
Motor LeftMotor(LEFT_MOTOR_LEN_PIN, LEFT_MOTOR_REN_PIN, LEFT_MOTOR_PWM_PIN);

Encoder RightEncoder(RIGHT_ENC_PIN_B, RIGHT_ENC_PIN_A); //it is inverted to get a right rotation sign
Encoder LeftEncoder(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);

LPF Ch_1_lpf(RECEIVER_LPF_CUT_OFF_FREQ);
LPF Ch_2_lpf(RECEIVER_LPF_CUT_OFF_FREQ);

LPF RightRPM_lpf(ENC_LPF_CUT_OFF_FREQ);
LPF LeftRPM_lpf(ENC_LPF_CUT_OFF_FREQ);

LPF dlpf(1);

pidIr RightMotorPID(KP_RIGHT_MOTOR, KI_RIGHT_MOTOR, KD_RIGHT_MOTOR);
pidIr LeftMotorPID(KP_LEFT_MOTOR, KI_LEFT_MOTOR, KD_LEFT_MOTOR);

//RC_Receiver receiver(PIN_CH_1, PIN_CH_2, PIN_CH_3, PIN_CH_4, PIN_CH_5, PIN_CH_6, PIN_CH_7, PIN_CH_8);

struct magnetometer {
  int x_msb;
  int x_lsb;
  int z_msb;
  int z_lsb;
  int y_msb;
  int y_lsb;

  float hx;
  float hz;
  float hy;
};

magnetometer magnetometer;

void callbackRA(){RightEncoder.doEncoderA();}
void callbackRB(){RightEncoder.doEncoderB();}
void callbackLA(){LeftEncoder.doEncoderA();}
void callbackLB(){LeftEncoder.doEncoderB();}

byte current_channel = 1;
uint16_t receiver_ch_value[9]; //PIN_CH_1 --> receiver_ch_value[1], and so on.
uint16_t receiver_ch_filtered[9]; //PIN_CH_1 --> receiver_ch_value[1], and so on.

float right_rpm_filtered;
float left_rpm_filtered;

float heading = 0;
float heading_filtered = 0;

int move_value;
int turn_value;
float right_rpm_target = 0;
float left_rpm_target = 0;
int right_pwm = 0;
int left_pwm = 0;
uint16_t failsafe = DISARMED;

// Vehicle Pose or State
float pose_x = 0;           // in cm
float pose_y = 0;           // in cm
float pose_theta = 0;       // in rad
float velocity_right = 0;   // in cm/s
float velocity_left = 0;    // in cm/s

unsigned long time_now = 0;
unsigned long time_last = 0;
float dt;

Servo camServo;
int servo_pos = MAX_SERVO_POS;

//ROS Communication
ros::NodeHandle nh;

// Varible for hardware command
uint8_t movement_command_ = 0;
uint8_t cam_angle_command_ = 0;
float right_motor_speed_ = 0;
float left_motor_speed_ = 0;

int a = 0;
int b = 0;

// Callback function that handles data subscribing
void callback_function( const slam_itbdelabo::HardwareCommand& msg){
  movement_command_ = msg.movement_command;
  cam_angle_command_ = msg.cam_angle_command;
  right_motor_speed_ = msg.right_motor_speed;
  left_motor_speed_ = msg.left_motor_speed;

  String OmegaString = String(String(dt) + ", Left RPM: " + String(left_motor_speed_,3) + " RPM, Right RPM: " + String(right_motor_speed_,3) + " RPM.");
  char OmegaInfo[100]; OmegaString.toCharArray(OmegaInfo, 100);
  nh.loginfo(OmegaInfo);
}

// Create subscriber for hardware command info
ros::Subscriber<slam_itbdelabo::HardwareCommand> sub("hardware_command", callback_function);

void setup(){
    Serial.begin(57600);
    Wire.begin();

    //Initiate ROS node
    nh.initNode();
    nh.subscribe(sub);
    
    RightMotor.begin();
    LeftMotor.begin();
    
    setupPinReceiver();

    RightEncoder.start(callbackRA, callbackRB);
    LeftEncoder.start(callbackLA, callbackLB);

    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    camServo.write(servo_pos);
    camServo.attach(CAM_SERVO);

    debugHeader();

    delay(2000);
}

void loop(){
    time_now = millis();
    if(time_now - time_last >= LOOP_TIME){
        dt = time_now - time_last;
        getReceiverSignal();
        
        receiver_ch_filtered[1] = Ch_1_lpf.filter(receiver_ch_value[1], dt);
        receiver_ch_filtered[2] = Ch_2_lpf.filter(receiver_ch_value[2], dt);

        //Calculate the robot position and velocity
        calculatePose();

        update_failsafe();
        update_cmd();
        
        time_last = time_now;
        debug();
        nh.spinOnce();
    }
}

void setupPinReceiver(){
    pinMode(PIN_CH_1, INPUT);
    pinMode(PIN_CH_2, INPUT);
    pinMode(PIN_CH_3, INPUT);
    pinMode(PIN_CH_4, INPUT);
    pinMode(PIN_CH_5, INPUT);
    pinMode(PIN_CH_6, INPUT);
    pinMode(PIN_CH_7, INPUT);
    pinMode(PIN_CH_8, INPUT);
}

void getReceiverSignal() {
    receiver_ch_value[current_channel] = pulseIn(getChannelPin(current_channel), HIGH, PERIOD_TIME);

    // Constrain the channel value within a specific range if needed
    receiver_ch_value[current_channel] = constrain(receiver_ch_value[current_channel], 1000, 2000);

    // Move to the next channel for the next loop iteration
    current_channel++;
    if (current_channel > NUM_CH) {
        current_channel = 1; // Reset to the first channel if all channels are processed
    }
}


int getChannelPin(byte channel) {
    switch (channel) {
        case 1:
            return PIN_CH_1;
        case 2:
            return PIN_CH_2;
        case 3:
            return PIN_CH_3;
        case 4:
            return PIN_CH_4;
        case 5:
            return PIN_CH_5;
        case 6:
            return PIN_CH_6;
        case 7:
            return PIN_CH_7;
        case 8:
            return PIN_CH_8;
        default:
            return -1;
    }
}

void update_failsafe(){
  // It is chosen because if RC is disconnected, the value is 0
  if(receiver_ch_value[4] > 1500){
    failsafe = ARMED;
  } else{
    failsafe = DISARMED;
  }
}

void update_cmd(){
  if(failsafe == DISARMED){ // Disarmed condition
    right_pwm = 0;
    left_pwm = 0;
    vehicle_stop();
    
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  } else{ // Armed condition
    if(receiver_ch_value[3] < 1400){
      /*
      //----------------------------Open loop-----------------------------//
      move_value = tuneReceiverSignaltoRPM(receiver_ch_filtered[1], MAX_PWM);
      turn_value = tuneReceiverSignaltoRPM(receiver_ch_filtered[2], MAX_PWM);
      
      right_pwm = move_value - turn_value;
      left_pwm = move_value + turn_value;
      //------------------------------------------------------------------//
      */

      
      //---------------------------Close loop-----------------------------//
      //move_value = tuneReceiverSignaltoRPM(receiver_ch_filtered[1], MAX_RPM_MOVE);
      //turn_value = tuneReceiverSignaltoRPM(receiver_ch_filtered[2], MAX_RPM_TURN);

      
      if(receiver_ch_filtered[1] >= 1500 + PWM_THRESHOLD){
          move_value = map(receiver_ch_filtered[1], 1500 + PWM_THRESHOLD, 2000, 0, MAX_RPM_MOVE);
      } else if(receiver_ch_filtered[1] <= 1500 - PWM_THRESHOLD){
          move_value = map(receiver_ch_filtered[1], 1500 - PWM_THRESHOLD, 1000, 0, -MAX_RPM_MOVE+90);
      } else {
          move_value = 0;
      }

      if(receiver_ch_filtered[2] >= 1450 + PWM_THRESHOLD){
          turn_value = map(receiver_ch_filtered[2], 1450 + PWM_THRESHOLD, 1950, 0, MAX_RPM_TURN+80);
      } else if(receiver_ch_filtered[2] <= 1450 - PWM_THRESHOLD){
          turn_value = map(receiver_ch_filtered[2], 1450 - PWM_THRESHOLD, 1000, 0, -MAX_RPM_TURN);
      } else {
          turn_value = 0;
      }
      
      right_rpm_target = move_value - turn_value;
      left_rpm_target = move_value + turn_value;
      
      if(right_rpm_target == 0 && left_rpm_target == 0){
          right_pwm = 0;
          left_pwm = 0;
          vehicle_stop();
      } else if (right_rpm_target == 0 && left_rpm_target != 0) {
          right_pwm = 0;
          RightMotorPID.reset();
          left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, 10);
      } else if (right_rpm_target != 0 && left_rpm_target == 0) {
          left_pwm = 0;
          LeftMotorPID.reset();
          right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, 10);
      } else {
          right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, 10);
          left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, 10);
      }
      //------------------------------------------------------------------//
      
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
    } else {
      // Part to control vehicle heading based on the target position
      right_rpm_target = right_motor_speed_;
      left_rpm_target = left_motor_speed_;

      if(right_rpm_target == 0 && left_rpm_target == 0){
          right_pwm = 0;
          left_pwm = 0;
          vehicle_stop();
      } else if (right_rpm_target == 0 && left_rpm_target != 0) {
          right_pwm = 0;
          RightMotorPID.reset();
          left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, 10);
      } else if (right_rpm_target != 0 && left_rpm_target == 0) {
          left_pwm = 0;
          LeftMotorPID.reset();
          right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, 10);
      } else {
          right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, 10);
          left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, 10);
      }
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, LOW);
    }
    vehicleGo(right_pwm, left_pwm);
    write_servo();
  }
}

int tuneReceiverSignaltoRPM(int receiver_signal, int max_rpm){
    if(receiver_signal >= 1500 + PWM_THRESHOLD){
        return map(receiver_signal, 1500 + PWM_THRESHOLD, 2000, 0, max_rpm);
    } else if(receiver_signal <= 1500 - PWM_THRESHOLD){
        return map(receiver_signal, 1500 - PWM_THRESHOLD, 1000, 0, -max_rpm);
    } else {
        return 0;
    }
}

void vehicle_stop(){
    RightMotor.stop();
    LeftMotor.stop();
    resetPID();
}

void vehicleGo(int right_pwm, int left_pwm){
    RightMotor.rotate(right_pwm);
    LeftMotor.rotate(left_pwm);
}

void resetPID(){
    RightMotorPID.reset();
    LeftMotorPID.reset();
}

void calculatePose(){
    float delta_angle_right = RightEncoder.getDeltaRad();
    float delta_angle_left = LeftEncoder.getDeltaRad();

    pose_x = pose_x + WHEEL_RADIUS/2.0 * (delta_angle_right + delta_angle_left) * sin(pose_theta);
    pose_y = pose_y + WHEEL_RADIUS/2.0 * (delta_angle_right + delta_angle_left) * cos(pose_theta);
    pose_theta = pose_theta + (delta_angle_right - delta_angle_left) * WHEEL_RADIUS/WHEEL_DISTANCE;

    pose_theta = wrapAngleRadian(pose_theta);

    RightEncoder.measureOmega();
    LeftEncoder.measureOmega();

    right_rpm_filtered = RightRPM_lpf.filter(RightEncoder.getOmegaRPM(), dt);
    left_rpm_filtered = LeftRPM_lpf.filter(LeftEncoder.getOmegaRPM(), dt);

    velocity_right = right_rpm_filtered * PI/30.0 * WHEEL_RADIUS;
    velocity_left = left_rpm_filtered * PI/30.0 * WHEEL_RADIUS;
}

float wrapAngleDegree(float value){
    if(value >= 2*360){
        return wrapAngleDegree(value - 360);
    } else if(value < 0){
        return wrapAngleDegree(value + 360);
    } else {
        return value;
    }
}

float wrapAngleRadian(float value){
    if(value >= 2*PI){
        return wrapAngleRadian(value - 2 * PI);
    } else if(value < 0){
        return wrapAngleRadian(value + 2 * PI);
    } else {
        return value;
    }
}

void write_hmc5983(int reg, int val){
    Wire.beginTransmission(HMC5983_ADDRESS);
    Wire.write(0x3C);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void read_hmc5983(int reg){
    Wire.beginTransmission(HMC5983_ADDRESS);
    Wire.write(0x3D);
    Wire.write(reg);
    Wire.endTransmission();
}

void get_heading(){
    write_hmc5983(0x00, 0x10);
    write_hmc5983(0x01, 0x20);
    write_hmc5983(0x02, 0x01);
    read_hmc5983(0x03);
  
    Wire.requestFrom(HMC5983_ADDRESS, 6);
    while(Wire.available()){
        magnetometer.x_msb = Wire.read();
        magnetometer.x_lsb = Wire.read();
        magnetometer.z_msb = Wire.read();
        magnetometer.z_lsb = Wire.read();
        magnetometer.y_msb = Wire.read();
        magnetometer.y_lsb = Wire.read();
    }

    magnetometer.hx = (magnetometer.x_msb << 8) + magnetometer.x_lsb;
    magnetometer.hz = (magnetometer.z_msb << 8) + magnetometer.z_lsb;
    magnetometer.hy = (magnetometer.y_msb << 8) + magnetometer.y_lsb;

    if(magnetometer.hx > 0x07FF) magnetometer.hx = 0xFFFF - magnetometer.hx;
    if(magnetometer.hz > 0x07FF) magnetometer.hz = 0xFFFF - magnetometer.hz;
    if(magnetometer.hy > 0x07FF) magnetometer.hy = 0xFFFF - magnetometer.hy;

    heading = atan2(magnetometer.hy, magnetometer.hx) * 180 / PI; //in deg
    heading_filtered = dlpf.filter(heading, dt);
}

void write_servo(){
  // RC mode
  if (receiver_ch_value[3] < 1600) {
    if (receiver_ch_value[5] >= 980 && receiver_ch_value[5] <= 2020){
      servo_pos = map(receiver_ch_value[5], 980, 2020, MIN_SERVO_POS + 5, MAX_SERVO_POS - 5);
      camServo.write(servo_pos);
    } else if (receiver_ch_value[5] > 2020) {
      // most up cam position
      servo_pos = MAX_SERVO_POS;
      camServo.write(servo_pos);
    } else {
      // most straight cam position
      servo_pos = MIN_SERVO_POS;
      camServo.write(servo_pos);
    }
  }
  // PC Mode (Auto)
  else {
    if (cam_angle_command_ == 1) {
      // Up
      servo_pos = servo_pos + INCREMENT_POS;
      servo_pos = min(servo_pos, MAX_SERVO_POS);
      camServo.write(servo_pos);
    }
    else if (cam_angle_command_ == 2) {
      // Down
      servo_pos = servo_pos - INCREMENT_POS;
      servo_pos = max(servo_pos, MIN_SERVO_POS);
      camServo.write(servo_pos);
    }
  }
}

void debugHeader(){
    #ifdef RECEIVER_RAW
    Serial.print(F("Ch1:")); Serial.print("\t");
    Serial.print(F("Ch2:")); Serial.print("\t");
    Serial.print(F("Ch3:")); Serial.print("\t");
    Serial.print(F("Ch4:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_PULSE
    Serial.print(F("Right_Pulse:")); Serial.print("\t");
    Serial.print(F("Left_Pulse:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_DEGREE
    Serial.print(F("Right_Deg:")); Serial.print("\t");
    Serial.print(F("Left_Deg:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_RADIAN
    Serial.print(F("Right_Rad:")); Serial.print("\t");
    Serial.print(F("Left_Rad:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_REVOLUTION
    Serial.print(F("Right_Rev:")); Serial.print("\t");
    Serial.print(F("Left_Rev:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_PPS
    Serial.print(F("Right_PPS:")); Serial.print("\t");
    Serial.print(F("Left_PPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_DPS
    Serial.print(F("Right_DPS:")); Serial.print("\t");
    Serial.print(F("Left_DPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPS
    Serial.print(F("Right_RPS:")); Serial.print("\t");
    Serial.print(F("Left_RPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPM
    Serial.print(F("Right_RPM:")); Serial.print("\t");
    Serial.print(F("Left_RPM:")); Serial.print("\t");
    #endif

    #ifdef TARGET_RPM
    Serial.print(F("Right_Target")); Serial.print("\t");
    Serial.print(F("Left_Target")); Serial.print("\t");
    #endif

    #ifdef PWM_RESPONSE
    Serial.print(F("Right_PWM")); Serial.print("\t");
    Serial.print(F("Left_PWM")); Serial.print("\t");
    #endif

    #ifdef ERROR_PID
    Serial.print(F("Error_Right")); Serial.print("\t");
    Serial.print(F("Error_Left")); Serial.print("\t");
    #endif

    #ifdef SUM_ERROR_PID
    Serial.print(F("Sum_Error_Right")); Serial.print("\t");
    Serial.print(F("Sum_Error_Left")); Serial.print("\t");
    #endif

    #ifdef VEHICLE_POSITION
    Serial.print(F("Vehicle_X_Pose")); Serial.print("\t");
    Serial.print(F("Vehicle_Y_Pose")); Serial.print("\t");
    Serial.print(F("Vehicle_Theta")); Serial.print("\t");
    #endif

    #ifdef VEHICLE_SPEED
    Serial.print(F("Vehicle_Speed_Right")); Serial.print("\t");
    Serial.print(F("Vehicle_Speed_Left")); Serial.print("\t");
    #endif

    #ifdef MOTOR_PULSE_DIFFERENCE
    Serial.print(F("Right_Pulse_diffference")); Serial.print("\t");
    Serial.print(F("Left_Pulse_diffference")); Serial.print("\t");
    #endif

    #ifdef EKF_DATA
    Serial.print(F("Right_Wheel_Speed")); Serial.print(",");
    Serial.print(F("Left_Wheel_Speed")); Serial.print(",");
    Serial.print(F("EKF_State")); Serial.print(",");
    #endif

    Serial.println();
}

void debug(){
    #ifdef RECEIVER_RAW
    Serial.print(receiver_ch_value[1]); Serial.print("\t");
    Serial.print(receiver_ch_value[2]); Serial.print("\t");
    Serial.print(receiver_ch_value[3]); Serial.print("\t");
    Serial.print(receiver_ch_value[4]); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_PULSE
    Serial.print(RightEncoder.getPulse()); Serial.print("\t");
    Serial.print(LeftEncoder.getPulse()); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_DEGREE
    Serial.print(RightEncoder.getAngleDeg()); Serial.print("\t");
    Serial.print(LeftEncoder.getAngleDeg()); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_RADIAN
    Serial.print(RightEncoder.getAngleRad()); Serial.print("\t");
    Serial.print(LeftEncoder.getAngleRad()); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_REVOLUTION
    Serial.print(RightEncoder.getAngleRev()); Serial.print("\t");
    Serial.print(LeftEncoder.getAngleRev()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_PPS
    Serial.print(RightEncoder.getOmegaPPS()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaPPS()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_DPS
    Serial.print(RightEncoder.getOmegaDPS()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaDPS()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPS
    Serial.print(RightEncoder.getOmegaRPS()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaRPS()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPM
    Serial.print(right_rpm_filtered); Serial.print("\t");
    Serial.print(left_rpm_filtered); Serial.print("\t");
    #endif

    #ifdef TARGET_RPM
    Serial.print(right_rpm_target); Serial.print("\t");
    Serial.print(left_rpm_target); Serial.print("\t");
    #endif

    #ifdef PWM_RESPONSE
    Serial.print(right_pwm/2.0); Serial.print("\t");
    Serial.print(left_pwm/2.0); Serial.print("\t");
    #endif

    #ifdef ERROR_PID
    Serial.print(RightMotorPID.getError()); Serial.print("\t");
    Serial.print(LeftMotorPID.getError()); Serial.print("\t");
    #endif

    #ifdef SUM_ERROR_PID
    Serial.print(RightMotorPID.getSumError()); Serial.print("\t");
    Serial.print(LeftMotorPID.getSumError()); Serial.print("\t");
    #endif

    #ifdef VEHICLE_POSITION
    Serial.print(pose_x); Serial.print("\t");
    Serial.print(pose_y); Serial.print("\t");
    Serial.print(pose_theta/PI*180.0); Serial.print("\t");
    #endif
    
    #ifdef VEHICLE_SPEED
    Serial.print(velocity_right); Serial.print("\t");
    Serial.print(velocity_left); Serial.print("\t");
    #endif

    #ifdef MOTOR_PULSE_DIFFERENCE
    Serial.print(RightEncoder.getDeltaPulse()); Serial.print("\t");
    Serial.print(LeftEncoder.getDeltaPulse()); Serial.print("\t");
    #endif

    #ifdef EKF_DATA
    Serial.print(velocity_right); Serial.print(",");
    Serial.print(velocity_left); Serial.print(",");
    #endif

    Serial.println();
}
