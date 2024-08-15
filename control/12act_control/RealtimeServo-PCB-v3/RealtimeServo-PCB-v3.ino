#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

// https://github.com/nanopb/nanopb/tree/master

#define NUM_MOTORS 12
#define MY_ID 0

//######################################## Timer ###################
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

// Create the three motor shield object I2C address input
Adafruit_MotorShield MC1 = Adafruit_MotorShield(0x60);
Adafruit_MotorShield MC2 = Adafruit_MotorShield(0x61);
Adafruit_MotorShield MC3 = Adafruit_MotorShield(0x62);

// Create 3 motor objects
// 1 - M3; 2 - M4; 3 - M1; 4 - M2
Adafruit_DCMotor *MC1_M1 = MC1.getMotor(3); // M1
Adafruit_DCMotor *MC1_M2 = MC1.getMotor(4); // M2
Adafruit_DCMotor *MC1_M3 = MC1.getMotor(1); // M3
Adafruit_DCMotor *MC1_M4 = MC1.getMotor(2); // M4

Adafruit_DCMotor *MC2_M1 = MC2.getMotor(3); // M1
Adafruit_DCMotor *MC2_M2 = MC2.getMotor(4); // M2
Adafruit_DCMotor *MC2_M3 = MC2.getMotor(1); // M3
Adafruit_DCMotor *MC2_M4 = MC2.getMotor(2); // M4

Adafruit_DCMotor *MC3_M1 = MC3.getMotor(3); // M1
Adafruit_DCMotor *MC3_M2 = MC3.getMotor(4); // M2
Adafruit_DCMotor *MC3_M3 = MC3.getMotor(1); // M3
Adafruit_DCMotor *MC3_M4 = MC3.getMotor(2); // M4

// create array of pointers to motor objects
Adafruit_DCMotor* motors[NUM_MOTORS] = {MC1_M1, MC1_M2, MC1_M3, MC1_M4,
                                        MC2_M1, MC2_M2, MC2_M3, MC2_M4,
                                        MC3_M1, MC3_M2, MC3_M3, MC3_M4
                                       };

// create object to access off-board analog-to-digital converter
Adafruit_ADS1015 ADC1; //ADC1 // ADC2
Adafruit_ADS1015 ADC2;
Adafruit_ADS1015 ADC3;

Adafruit_ADS1015* adcs[NUM_MOTORS] = {&ADC1, &ADC1, &ADC1, &ADC1,
                                      &ADC2, &ADC2, &ADC2, &ADC2,
                                      &ADC3, &ADC3, &ADC3, &ADC3,
                                     };
int channels[NUM_MOTORS] = {0, 1, 2, 3,
                            0, 1, 2, 3,
                            0, 1, 2, 3
                           };

//#################################### GLOBAL VARIABLES ##################################//

/* TBD if needed - Existed in the code but wasn't used by anything
  float max_motor_speed[NUM_MOTORS] =     {15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0};
  float max_motor_position[NUM_MOTORS] =  {19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5};
  float desired_motor_speed[1];
  int sampleTime = 0;
  float time_left = max(durations[current_trajectory_point] - last_time, 0.1);
  const int numChars = 3000;
*/

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Misc
float pi = 3.1415926535;

// Calculate based on max input size expected for one command
#define MAX_INPUT_SIZE 20000

// Sending and Receiving Information
char inputString[MAX_INPUT_SIZE + 1];
bool newData = false;
int currentStringLength = 0;
int ndx = 0;
bool recvInProgress = false;
char startMarker = '<';
char endMarker = '>';

// Stop Command Global
bool stop_flag = false;

// Conversions ******************************************************     CHECK THIS      ***********************************
#define ADC2M 0.012 //ADC reading to pos in [mm]
#define VEL2CMD 17.0 // vel [mm/s] to motor command val [x/+-255]

// Displaying Information
String joint_states;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Position and Velocity Information
int motor_val[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // pre-conversion to mm
float joint_positions[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};       // mm
float joint_velocities[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      // mm/s
float last_joint_positions[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // mm

// Error Information
float last_joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};     // mm
float joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};          // mm

float position_threshold = 1.0;   // mm  //***********************      CHANGE THIS?      ***********************************

// Trajectory Information
float desired_joint_positions[1][NUM_MOTORS]; // Typically num_trajectory_points x NUM_MOTORS
float durations[1];
int num_trajectory_points = 0;
int current_trajectory_point = 0;
bool position_trajectory = false;

float reset_pos = 0.5; // mm //**********************************      CHANGE THIS!      ***********************************

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Time Information
float last_time = 0.0;                // s
unsigned long current_arduino_time;   // ms
unsigned long last_arduino_time;      // ms
float time_elapsed;                   // s

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// PID
float k_p = 3.0; // 2.5 // 3.0; // 0.2 // 5.0 // 2.0 
float k_i = 0.5;
float k_d = 0.02;

float _cmdPos_error[NUM_MOTORS];
float _cmdPos_integral_error[NUM_MOTORS];
float _cmdPos_derivative_error[NUM_MOTORS];

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// INTEGRATING CHRIS CODE - CHANGES MAY NEED TO BE MADE HERE

// Max values
float tf_max = 0;       // s - maximum completion time needed to move an actuator

// Commands
float _joint_positionsCmd[NUM_MOTORS] =   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float _joint_velocitiesCmd[NUM_MOTORS] =  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float _refCmd[NUM_MOTORS] =               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float _motorCmd[NUM_MOTORS] =             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Receive Data
void recvWithStartEndMarkers() {
  char rc;

//  Serial.println();
//  Serial.println("RECIEVING");
  
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        inputString[ndx] = rc;
        ndx++;
      }
      else {
        inputString[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        currentStringLength = ndx;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
//  Serial.println(inputString);
//  Serial.println("DONE RECIEVING");
//  Serial.println();
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// STOP COMMAND
// Immediately halts everything by setting all the motor control pins to LOW and sets the stop_flag to true.
// Remember to publish a false msg to restart the robot.

void stop(bool stop_msg) {
  // If the stop_msg contains true, stop all of the motors
  if (stop_msg)
  {
    // set stop_flag to true
    stop_flag = true;

    // Turn off all motors
    for (int i = 0; i < NUM_MOTORS; i++)
    {
      if (motors[i] != NULL) {
        motors[i]->run(RELEASE);
      }
    }
  }
  // Otherwise if the stop_msg contains false, set the stop_flag back to false
  else
  {
    stop_flag = false;
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// RESET COMMAND
// Resets the robot's desired positions to the default positions.

void resetJoints() {
  num_trajectory_points = 1;
  durations[0] = 4.0;
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    desired_joint_positions[0][i] = reset_pos;
  }

  position_trajectory = true;
  current_trajectory_point = 0;
  last_time = 0.0;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// STATE UPDATES

// Position Update
void updateJointPositions()
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    if (motors[i] != NULL)
    {
      motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]);
      joint_positions[i] = motor_val[i] * ADC2M; // 3.0 * 0.02 / 5.0; // 1 bit = 2mV @ 4.096 FSR 0.02 mm stroke 3.3V range // 100mm / 1650 analog reading
    }
  }
}

// Previous Position Update
void updatePreviousJointPositions()
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    if (motors[i] != NULL)
    {
      last_joint_positions[i] = joint_positions[i];
    }
  }
}

// Time Update
void updateTime() {
  // Update time
  current_arduino_time = millis();
  time_elapsed = float(current_arduino_time - last_arduino_time) / 1000.0;
  last_time += time_elapsed;

  // Store time
  last_arduino_time = current_arduino_time;
}

// Velocity Update
void updateVelocity() {
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    joint_velocities[i] = (joint_positions[i] - last_joint_positions[i]) / time_elapsed;
  }
}

// State Update
void updateState() {
  updatePreviousJointPositions();
  updateTime();
  updateJointPositions();
  updateVelocity(); 
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// POSITION TRAJECTORY
// Stores the Position Trajectory to desired_joint_positions
void positionTrajectory()
{
  String received = "p ";

  Serial.println();
  Serial.print(received);

  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(inputString, ",");     // get the first part - the string
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  num_trajectory_points = atoi(strtokIndx);

  for (int i = 0; i < num_trajectory_points; i++)
  {
    received = "p ";
    strtokIndx = strtok(NULL, ",");         // this continues where the previous call left off
    durations[i] = atof(strtokIndx);
    Serial.print("durations i");
    Serial.println(durations[i]);
    received += String(durations[i]) + " ";

    for (int j = 0; j < NUM_MOTORS; j++)
    {
      strtokIndx = strtok(NULL, ",");       // this continues where the previous call left off
      desired_joint_positions[i][j] = atof(strtokIndx);
      received += String(desired_joint_positions[i][j], 4) + " ";
    }

    Serial.print("Trajectory Point ");
    Serial.print(i);
    Serial.print(": \t");
    Serial.println(received);
  }
  position_trajectory = true;
  current_trajectory_point = 0;
  last_time = 0.0;

  Serial.print("Number of trajectory points \t");
  Serial.println(num_trajectory_points);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// MOVE DELTA USING P CONTROL
// Each actuator moves at its own rate

void moveDeltaPosition()
{
  bool reached_point = true;
  Serial.println();
  
  Serial.print("E, ");
  // Check if any of the motors have not reached their desired position
  for (int idx = 0; idx < NUM_MOTORS; idx++)
  {
    if (motors[idx] != NULL)
    {
      updateJointPositions();
      joint_errors[idx] = joint_positions[idx] - desired_joint_positions[current_trajectory_point][idx];
      
      Serial.print(joint_errors[idx], 4);
      Serial.print(",");

      if (fabs(joint_errors[idx]) > position_threshold)
      {
        reached_point = false;
      } 
      // Stop motor if it's reached its desired point (within the threshold)
      else 
      {
        motors[idx]->setSpeed(0);
        motors[idx]->run(RELEASE);
      }
    }
  }

  Serial.println();
  
  // If all motors have reached their desired position, move onto the next trajectory
  if (reached_point)
  {
    Serial.println("reached point, onto next trajectory");
    current_trajectory_point += 1;
  }

  // P control if it has not reached the desired point yet
  if (current_trajectory_point < num_trajectory_points)
  {
    for (int i = 0; i < NUM_MOTORS; i++)
    {
      if (motors[i] != NULL)
      {
        // Move forwards
        if (joint_errors[i] > position_threshold)
        {
          //  Serial.println("Forwarding...");
          float pid = k_p * joint_errors[i];
          int motor_speed = (int)(min(max(0.0, pid), 1.0) * 255.0); //change it to use duration input
          // Serial.print(motor_speed);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(FORWARD);
        }
        // Move backwards
        else if (joint_errors[i] < -position_threshold)
        {
          //  Serial.println("Backwarding...");
          float pid = k_p * joint_errors[i];
          int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -255.0);
          // Serial.print(motor_speed);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(BACKWARD);
        }
        // Do nothing
        else
        {
          //  Serial.println("Doing nothing...");
          motors[i]->setSpeed(0);
          motors[i]->run(RELEASE);
        }
      }
    }
  }
  // Do nothing if it has reached the desired point 
  else
  {
    for (int i = 0; i < NUM_MOTORS; i++)
    {
      if (motors[i] != NULL) {
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
      }
    }
    Serial.println("end of all available trajectories");
    position_trajectory = false; //finished moving all commands
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// MOTOR COMMANDS

void cmdPos(float qCmd[NUM_MOTORS])
{
  for (int i = 0; i < NUM_MOTORS; i++) {
    _joint_positionsCmd[i] = qCmd[i];
  }
}

void cmdVel(float q_dotCmd[NUM_MOTORS])
{
//  Serial.println();
//  Serial.println("cmdVel");
  for (int i = 0; i < NUM_MOTORS; i++) {
    _joint_velocitiesCmd[i] = q_dotCmd[i];
//    Serial.print("\t ");
//    Serial.print(q_dotCmd[i]);
    _refCmd[i] = q_dotCmd[i] * VEL2CMD;
  }
//  Serial.println();
}

void positional_PID(float PID_Cmd[NUM_MOTORS]) {
  for (int i=0; i<NUM_MOTORS; i++) {
    float error = _joint_positionsCmd[i] - joint_positions[i];
    _cmdPos_integral_error[i] = _cmdPos_integral_error[i] + error * (time_elapsed);
    _cmdPos_derivative_error[i] = (error - _cmdPos_error[i]) / (time_elapsed);
    _cmdPos_error[i] = error;
  
    PID_Cmd[i] = (k_p*_cmdPos_error[i] + 
                  k_i*_cmdPos_integral_error[i] + 
                  k_d*_cmdPos_derivative_error[i]);
  }
}

void send_motorCmd()
{
//  Serial.println();
//  Serial.println("SEND MOTOR COMMAND");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    int motor_cmd_val = _motorCmd[i];
    motor_cmd_val = min(max(motor_cmd_val, -255), 255); // Clip to safe bounds

//    Serial.print("\tMotor ");
//    Serial.print(i);
//    Serial.print(" \t\t_motorCmd ");
//    Serial.print(_motorCmd[i]);
//    Serial.print(" \t\t Clipped value ");
//    Serial.print(motor_cmd_val);
//    Serial.print(" \t\t speed ");
//    Serial.println((int)(abs(motor_cmd_val)));

    int motor_threshold = 0;
//    Serial.println(motor_cmd_val);
    motors[i]->setSpeed((int)(abs(motor_cmd_val)));
//    motors[i]->setSpeed(100);
    if (motor_cmd_val > motor_threshold) { // 20 threshold?
//      Serial.println("FORWARD");
      motors[i]->run(BACKWARD);
    } else if (motor_cmd_val < motor_threshold) {
//      Serial.println("BACKWARD");
      motors[i]->run(FORWARD);
    } else {
//      Serial.println("RELEASE");
      motors[i]->run(RELEASE);
    }
  }
}

void driveController()
{
  updateState();

//  Serial.println();
//  Serial.println("DRIVE CONTROLLER");

  float PID_Cmd[NUM_MOTORS];
  positional_PID(PID_Cmd);

  for (int i = 0; i < NUM_MOTORS; i++) {
    float error = _joint_positionsCmd[i] - joint_positions[i];
//    float error = desired_joint_positions[current_trajectory_point][i] - joint_positions[i];
//    float p_command = P;

//    Serial.print("\t Motor ");
//    Serial.print(i);
//    Serial.print(" \t\t pos cmd ");
//    Serial.print(_joint_positionsCmd[i]);
//    Serial.print(" \t\t cur pos ");
//    Serial.print(joint_positions[i]);
//    Serial.print(" \t\t err ");
//    Serial.print(error);
//    Serial.print(" \t\t des pos ");
//    Serial.print(desired_joint_positions[current_trajectory_point][i]);
//    Serial.print("\t\t ref_cmd ");
//    Serial.print(_refCmd[i]);
//    Serial.print(" \t\t pid_command ");
//    Serial.println(PID_Cmd[i]);
    
    _motorCmd[i] = (int)(_refCmd[i] + PID_Cmd[i] * VEL2CMD); //TODO change 0.5?
  }
  send_motorCmd();
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TRAJECTORIES 

// Get maximum tf
void ramp_tf(float tf[NUM_MOTORS], float vmax, float amax)
{
  float tr;
  float tm;

  float delta_x;

  tf_max = 0;
  for (int idx = 0; idx < NUM_MOTORS; idx++)
  {
    delta_x = abs(desired_joint_positions[current_trajectory_point][idx] - joint_positions[idx]);
    
    tr = min(delta_x / vmax, vmax / amax);      
    tm = max(0, delta_x / vmax - vmax / amax);
    tf[idx] = 2 * tr + tm;

//    Serial.println();
//    Serial.print("Ramp tf calculation for motor ");
//    Serial.println(idx);
//    Serial.print("tf: ");
//    Serial.print(tf[idx]);
//    Serial.print(" \t tf_max prev: ");
//    Serial.print(tf_max);
//    Serial.print(" \t dx/vmax: ");
//    Serial.print(delta_x / vmax);
//    Serial.print(" \t vmax/amax: ");
//    Serial.print(vmax / amax);
//    Serial.print(" \t 0 \t dx/vmax - vmax/amax: ");
//    Serial.print(delta_x / vmax - vmax / amax);
//    Serial.print(" \t tf_max new: ");
    tf_max = max(tf_max, tf[idx]);
//    Serial.println(tf_max);
  }
}

// Modify qCmd with the x(t) values of a ramp traj
void ramp_pos(float t, float qCmd[NUM_MOTORS], float v[NUM_MOTORS], float amax_ref)
{ 
  float vmax;
  float amax;

//  Serial.println();
  
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    float delta_x = desired_joint_positions[current_trajectory_point][i] - joint_positions[i];
    
    if (delta_x < 0) {
      vmax = -v[i];
      amax = -amax_ref;
    } else {
      vmax = v[i];
      amax = amax_ref;
    }
    
    float tr = min(delta_x / vmax, vmax / amax);
    float tm = max(0, delta_x / vmax - vmax / amax);
  
//    Serial.print(i);
//    Serial.print("\t");
//    Serial.print(t);
//    Serial.print("\t");
//    Serial.print(tr);
//    Serial.print("\t");
//    Serial.print(tm);
//    Serial.print("\t");
//    Serial.print(joint_positions[i]);
//    Serial.print("\t");
    
    // Piece wise integration
    if (t < tr) {
//      Serial.println("C1");
      qCmd[i] = joint_positions[i] + 0.5 * amax * pow(t, 2);
    } else if (tr <= t & t < tr + tm) {
//      Serial.println("C2");
      qCmd[i] = joint_positions[i] + (0.5 * amax * pow(tr, 2)) + vmax * (t - tr);
    } else if (tr + tm <= t & t < 2 * tr + tm) {
//      Serial.println("C3");
      qCmd[i] = joint_positions[i] + (0.5 * amax * pow(tr, 2)) + (vmax * tm) + (vmax * (t - tm - tr) - 0.5 * amax * pow(t - tm - tr, 2));
    } else {
//      Serial.println("C4");
      qCmd[i] = desired_joint_positions[current_trajectory_point][i];
    }
  }
}

// Modify q_dotCmd with the v(t) values of a ramp traj
void ramp_vel(float t, float q_dotCmd[NUM_MOTORS], float v[NUM_MOTORS], float amax_ref)
{
  float vmax;
  float amax;

  float pure_PID_tolerance = 5.0;
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    float delta_x = desired_joint_positions[current_trajectory_point][i] - joint_positions[i];
    
    if (delta_x < 0) {
      vmax = -v[i];
      amax = -amax_ref;
    } else {
      vmax = v[i];
      amax = amax_ref;
    }
    
    float tr = min(delta_x / vmax, vmax / amax);
    float tm = max(0, delta_x / vmax - vmax / amax);
    
    // Piece wise integration
    if (t < tr) {
      q_dotCmd[i] = amax * t;
    } else if (tr <= t & t < tr + tm) {
      q_dotCmd[i] = vmax;
    } else if (tr + tm <= t & t < 2 * tr + tm) {
      q_dotCmd[i] = vmax - amax * (t - tr - tm);
    } else {
      q_dotCmd[i] = 0.0;
    }
    
    if (abs(delta_x) < pure_PID_tolerance) {
      q_dotCmd[i] = 0.0;
    }
  }
}

// Move to desired trajectories
void ramp2pos(float vmax, float amax)
{
  float v[NUM_MOTORS]; 
  float tf[NUM_MOTORS];
  
  updateState();

  ramp_tf(tf, vmax, amax); // get max time to move the actuators

//  Serial.println();
//  Serial.println("VELOCITES");
  for (int i = 0; i < NUM_MOTORS; i++) {
    v[i] = vmax * tf[i] / tf_max;
//    Serial.print(" \t Motor ");
//    Serial.print(i);
//    Serial.print(" \t v ");
//    Serial.println(v[i]);
  }

//  Serial.println();
//  Serial.println("RAMP2POS");

  float qCmd[NUM_MOTORS];
  float q_dotCmd[NUM_MOTORS];

  float _traj_t0 = millis() / 1000.0;
  float _traj_tf = tf_max;
  float _traj_t = 0;

  while (_traj_t < _traj_tf)
  {

//    Serial.println("In while loop-----------------");
    _traj_t = (millis() / 1000.0) - _traj_t0;

//    Serial.println("Ramp pos");
    ramp_pos(_traj_t, qCmd, v, amax);
//    Serial.println("Ramp vel");
    ramp_vel(_traj_t, q_dotCmd, v, amax);

//    Serial.println("CMD pos");
    cmdPos(qCmd);
//    Serial.println("CMD vel");
    cmdVel(q_dotCmd);
//    Serial.println("Drive controller");
    driveController();

    float q = joint_positions[0];
    float qq = qCmd[0];
    float q_dot = joint_velocities[0];
    float qq_dot = q_dotCmd[0];
//
//    Serial.println("DONE DRIVE CONTROLLER");
//
//    Serial.print("\t Cur time: ");
//    Serial.print(_traj_t); 
//    Serial.print("\t\t Tot time: ");
//    Serial.println(_traj_tf); 
//    Serial.print("\t q \t pos val: ");
//    Serial.print(q); 
//    Serial.print("\t\t qq \t pos cmd: ");
//    Serial.println(qq); 
//    Serial.print("\t qd \t vel val: ");
//    Serial.print(q_dot); 
//    Serial.print("\t\t qqd \t vel cmd: ");
//    Serial.println(qq_dot);
//    Serial.println("DELAY");
    delay(10);
  }
//  Serial.println("Exit while loop");
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// PERFORM TRAJECTORY

void executeWaypoints()
{
  bool reached_point = true;
  float vmax = 15.0;
  float amax = 15.0;

  Serial.println();
  Serial.print("Curr point ");
  Serial.print(current_trajectory_point);
  Serial.print(" Max pts ");
  Serial.println(num_trajectory_points);

  Serial.println();
  Serial.print("E, ");
  for (int idx = 0; idx < NUM_MOTORS; idx++)
  {
    if (motors[idx] != NULL)
    {
      updateJointPositions();
      joint_errors[idx] = joint_positions[idx] - desired_joint_positions[current_trajectory_point][idx];
//      Serial.print(desired_joint_positions[current_trajectory_point][idx]);
//      Serial.print(" ");
      Serial.print(joint_errors[idx], 4);
      Serial.print(",");
      if (fabs(joint_errors[idx]) > position_threshold)
      {
        reached_point = false;
      } else
      {
        motors[idx]->setSpeed(0);
        motors[idx]->run(RELEASE);
      }
    }
  }
  
  Serial.println();
  Serial.print("Checked errors, reached point ");
  Serial.println(reached_point);

  // If all motors have reached their desired position, move onto the next trajectory
  if (reached_point)
  {
    Serial.println("reached point, onto next trajectory");
    current_trajectory_point += 1;
  }

  if (current_trajectory_point < num_trajectory_points)
  {
    ramp2pos(vmax, amax);
  }
  else
  {
    for (int i = 0; i < NUM_MOTORS; i++)
    {
      if (motors[i] != NULL) {
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
      }
    }
    Serial.println("end of all available trajectories");
    position_trajectory = false; //finished moving all commands
  }
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// PRINTING

void publishJointStates()
{
  joint_states = "j,";

  for (int i = 0; i < NUM_MOTORS; i++)
  {
    joint_states += String(joint_positions[i], 4) + ","; //+ String(joint_velocities[i],4) + ",";
  }
  Serial.println(joint_states);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// SETUP CODE
void setup()
{
  Serial.begin(9600);
  delay(5000);

  // set all the base dc motor control pins to outputs
  MC1.begin();
  MC2.begin();
  MC3.begin();
  // start all the ADCs
  ADC3.begin(0x4A);
  ADC2.begin(0x49);
  ADC1.begin(0x48);

  ADC3.setGain(GAIN_ONE);
  ADC2.setGain(GAIN_ONE);
  ADC1.setGain(GAIN_ONE);

  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i]->setSpeed(0);
    motors[i]->run(RELEASE);
  }

  updateJointPositions();
  publishJointStates();
//  updatePreviousJointPositions();
  
  joint_states.reserve(200); // Reserve 200 Bytes for joint_states
  resetJoints();
  
  Serial.println("SETUP MOVING");
//    moveDeltaPosition();
//  executeWaypoints();
  ramp2pos(15.0, 12.5);
  Serial.println("SETUP FINISHED");
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// LOOP CODE
void loop()
{
  // Mode selection
  recvWithStartEndMarkers();
  Serial.print("new data tf ");
  Serial.println(newData);
  if (newData) {
    Serial.println(inputString);
    switch (inputString[0]) {
      // Stop Command
      case 's':
        stop(inputString[2] - '0');
        break;
      // Reset Command
      case 'r':
        resetJoints();
        break;
      // Position Command
      case 'p':
        positionTrajectory();
        break;
      default:
        // do nothing
        break;
    }
    newData = false;
  }

  // If the robot is not currently in the stop mode, move
  if (!stop_flag)
  {
    if (position_trajectory)
    {
//      moveDeltaPosition();
      Serial.print("num pts ");
      Serial.print(num_trajectory_points);
      Serial.println(" execute waypoints ");
      executeWaypoints();
    }
  }

  // Update state
  updateState();

  // Print joint states
  publishJointStates();

  // Pause prior to looping again
  delay(1);
}
