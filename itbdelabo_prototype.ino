#include <Wire.h>
#include <ros.h>
#include <follower/TargetState.h>

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
//#define MOTOR_SPEED_RPM
//#define TARGET_RPM
//#define PWM_RESPONSE
//#define VEHICLE_POSITION
//#define VEHICLE_SPEED
#define EKF_DATA


// Receiver PIN
#define PIN_CH_1 42
#define PIN_CH_2 44
#define PIN_CH_3 46
#define PIN_CH_4 48
#define PIN_CH_5 43
#define PIN_CH_6 45
#define PIN_CH_7 47
#define PIN_CH_8 49

// Motor PIN
#define RIGHT_MOTOR_REN_PIN 6
#define RIGHT_MOTOR_LEN_PIN 5
#define RIGHT_MOTOR_PWM_PIN 8

#define LEFT_MOTOR_REN_PIN 9
#define LEFT_MOTOR_LEN_PIN 10
#define LEFT_MOTOR_PWM_PIN 7

// Encoder PIN
#define RIGHT_ENC_PIN_A 11
#define RIGHT_ENC_PIN_B 12

#define LEFT_ENC_PIN_A A8
#define LEFT_ENC_PIN_B A9

// LED PIN
#define RED_LED  30
#define BLUE_LED 31

// Constants
#define LOOP_TIME 10                // in milliseconds
#define PERIOD_TIME 2*pow(10,6)     // in microseconds
#define RECEIVER_LPF_CUT_OFF_FREQ 1 // in Hertz (Hz)
#define ENC_LPF_CUT_OFF_FREQ 3      // in Hertz (Hz)
#define PWM_THRESHOLD 150           // in microseconds of receiver signal
#define MAX_RPM_MOVE 40             // in RPM for longitudinal movement
#define MAX_RPM_TURN 30             // in RPM for rotational movement
#define WHEEL_RADIUS 5.0            // in cm
#define WHEEL_DISTANCE 33.0         // in cm
#define DISTANCE 200                // in cm (maximum distance for ultrasonic)
#define MAX_PWM 200                 // saturation PWM for action control (0-255)
#define ARMED 0x00                  // armed condition
#define DISARMED 0x01               // disarmed condition
#define HMC5983_ADDRESS 0x1E        // magnetometer I2C address

#define KP_RIGHT_MOTOR 0.325
#define KI_RIGHT_MOTOR 0.0
#define KD_RIGHT_MOTOR 0.0

#define KP_LEFT_MOTOR 0.325
#define KI_LEFT_MOTOR 0.0
#define KD_LEFT_MOTOR 0.0

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

//ROS Communication
ros::NodeHandle nh;

// Varible for target position and distaance
uint16_t target_position_ = 0;
float target_distance_ = -1;

// Callback function that handles data subscribing
void callback_function( const follower::TargetState& msg){
  target_position_ = msg.target_position;
  target_distance_ = msg.target_distance;
}

// Create subscriber for target info
ros::Subscriber<follower::TargetState> sub("rover_command", callback_function);

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
        
        time_last = time_now;
        debug();
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

void getReceiverSignal(){
    receiver_ch_value[1] = pulseIn(PIN_CH_1, HIGH, PERIOD_TIME);
    receiver_ch_value[2] = pulseIn(PIN_CH_2, HIGH, PERIOD_TIME);
    receiver_ch_value[3] = pulseIn(PIN_CH_3, HIGH, PERIOD_TIME);
    receiver_ch_value[4] = pulseIn(PIN_CH_4, HIGH, PERIOD_TIME);
    receiver_ch_value[5] = pulseIn(PIN_CH_5, HIGH, PERIOD_TIME);
    receiver_ch_value[6] = pulseIn(PIN_CH_6, HIGH, PERIOD_TIME);
    receiver_ch_value[7] = pulseIn(PIN_CH_7, HIGH, PERIOD_TIME);
    receiver_ch_value[8] = pulseIn(PIN_CH_8, HIGH, PERIOD_TIME);

    for(int i = 1; i <= 8; i++){
      receiver_ch_value[i] = constrain(receiver_ch_value[i], 1000, 2000);
    }
}

void update_failsafe(){
  // It is chosen because if RC is disconnected, the value is 0
  if(receiver_ch_value[4]){
    failsafe = ARMED;
  } else{
    failsafe = DISARMED;
  }
}

void update_cmd(){
  if(failsafe == DISARMED){ // Disarmed condition
    right_pwm = 0;
    left_pwm = 0;
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  } else{ // Armed condition
    if(receiver_ch_value[3] < 1600){
      //----------------------------Open loop-----------------------------//
      move_value = tuneReceiverSignaltoRPM(receiver_ch_filtered[1], MAX_PWM);
      turn_value = tuneReceiverSignaltoRPM(receiver_ch_filtered[2], MAX_PWM);
      
      right_pwm = move_value - turn_value;
      left_pwm = move_value + turn_value;
      //------------------------------------------------------------------//

      /*
      //---------------------------Close loop-----------------------------//
      move_value = tuneReceiverSignaltoRPM(receiver_ch_filtered[1], MAX_RPM_MOVE);
      turn_value = tuneReceiverSignaltoRPM(receiver_ch_filtered[2], MAX_RPM_TURN);

      right_rpm_target = move_value + turn_value;
      left_rpm_target = move_value - turn_value;

      right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
      left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);
      //------------------------------------------------------------------//
      */
      
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
    } else {
      // Part to control vehicle heading based on the target position
      if (target_position_ == 1){
        rotate_left();
      } else if (target_position_ == 2){
        rotate_right();
      } else if (target_position_ == 3 && target_distance_ > DISTANCE){
        move_forward();
      } else {
        vehicle_stop();
      }
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, LOW);
    }
    // Write to motor
    vehicleGo(right_pwm, left_pwm);
  }
}

void rotate_left(){
  // Write down the algorithm so the vehicle rotate left
  right_pwm = 1.0*MAX_PWM;
  left_pwm = -1.0*MAX_PWM;
}

void rotate_right(){
  // Write down the algorithm so the vehicle rotate right
  right_pwm = -1.0*MAX_PWM;
  left_pwm = 1.0*MAX_PWM;
}

void move_forward(){
  // Write down the algorithm so the vehicle move forward
  right_pwm = 1.0*MAX_PWM;
  left_pwm = 1.0*MAX_PWM;
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

    pose_theta = wrapAngleFloatDegree(pose_theta);

    RightEncoder.measureOmega();
    LeftEncoder.measureOmega();

    right_rpm_filtered = RightRPM_lpf.filter(RightEncoder.getOmegaRPM(), dt);
    left_rpm_filtered = LeftRPM_lpf.filter(LeftEncoder.getOmegaRPM(), dt);

    velocity_right = right_rpm_filtered * PI/30.0 * WHEEL_RADIUS;
    velocity_left = left_rpm_filtered * PI/30.0 * WHEEL_RADIUS;
}

float wrapAngleFloatDegree(float value){
    if(value >= 2*360){
        return wrapAngleFloatDegree(value - 360);
    } else if(value < 0){
        return wrapAngleFloatDegree(value + 360);
    } else {
        return value;
    }
}

float wrapAngleFloatRadian(float value){
    if(value >= 2*PI){
        return wrapAngleFloatRadian(value - 2 * PI);
    } else if(value < 0){
        return wrapAngleFloatRadian(value + 2 * PI);
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
    Serial.print(ch_1_value); Serial.print("\t");
    Serial.print(ch_2_value); Serial.print("\t");
    Serial.print(ch_3_value); Serial.print("\t");
    Serial.print(ch_4_value); Serial.print("\t");
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
    Serial.print(right_pwm); Serial.print("\t");
    Serial.print(left_pwm); Serial.print("\t");
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
