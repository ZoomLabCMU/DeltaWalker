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

float max_motor_speed[NUM_MOTORS] = {0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015};
float max_motor_position[NUM_MOTORS] = {19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5, 19.5};

int motor_val[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float desired_joint_positions[1][NUM_MOTORS];

float joint_positions[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float joint_velocities[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float last_joint_positions[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float durations[1];
float desired_motor_speed[1];

float last_time = 0.0;
int num_trajectory_points = 0;
int current_trajectory_point = 0;
bool position_trajectory = false;

//#################################### GLOBAL VARIABLES ##################################//
const int numChars = 3000;
float pi = 3.1415926535;

// Calculate based on max input size expected for one command
#define MAX_INPUT_SIZE 900

char inputString[MAX_INPUT_SIZE + 1];
bool newData = false;
int currentStringLength = 0;
int ndx = 0;
bool recvInProgress = false;
char startMarker = '<';
char endMarker = '>';

unsigned long current_arduino_time;
unsigned long last_arduino_time;
float time_elapsed;
String joint_states;

int sampleTime = 0;

float position_threshold = 1.0; //1 mm threshold
float p = 0.2; //0.3; //400;//390.0;
float i_pid = 0.0;//0.1;//0.25;
float d = 0.0;//3.75;// 0.5

float last_joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float joint_errors[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float reset_pos = 15.0; //----------------------------------------------------------------------------------------------------------------------------------------------------------------

// INTEGRATING CHRIS CODE - CHANGES MAY NEED TO BE MADE HERE

float tf_max = 0;
float vmax_ref = 15.0; // mm/s -> SWITCH TO M/S???
float amax_ref = 15.0; // mm/s2 -> SWITCH TO M/S2???
float vmax = 15.0;
float amax = 15.0;

float qCmd[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float q_dotCmd[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float _joint_positionsCmd[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float _joint_velocitiesCmd[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float _refCmd[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float _motorCmd[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int traj_iter = 0;

bool go = true;

#define ADC2M 0.012 //ADC reading to pos in [mm]
#define VEL2CMD 17 // vel [mm/s] to motor command val [x/+-255]

void recvWithStartEndMarkers() {
  char rc;

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
}

// STOP COMMAND
// Stop Command Globals
bool stop_flag = false;

// Stop Command
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

// RESET COMMAND
// Reset Command Callback
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

void updateJointPositions()
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    if (motors[i] != NULL)
    {
      last_joint_positions[i] = joint_positions[i];
      motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]);
      joint_positions[i] = motor_val[i] * 3.0 * 0.02 / 5.0; // 1 bit = 2mV @ 4.096 FSR 0.02 mm stroke 3.3V range //* 0.0000303; // 100mm / 1650 analog reading
    }
  }
}

void updateTime() {
  // Update time
  current_arduino_time = millis();
  time_elapsed = float(current_arduino_time - last_arduino_time) / 1000.0;
  last_time += time_elapsed;

  // Store time
  last_arduino_time = current_arduino_time;
}

void updateVelocity() {
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    joint_velocities[i] = (joint_positions[i] - last_joint_positions[i]) / time_elapsed;
  }
}

void updateState() {
  updateJointPositions();
  updateTime();
  updateVelocity();
}

// Position Trajectory
// Stores the Position Trajectory to desired_joint_positions
void positionTrajectory()
{
  String received = "p ";

  Serial.println(received);

  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(inputString, ",");     // get the first part - the string
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  num_trajectory_points = atoi(strtokIndx);
  Serial.println(num_trajectory_points);
  for (int i = 0; i < num_trajectory_points; i++)
  {
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    durations[i] = atof(strtokIndx);
    received += String(durations[i]) + " ";

    for (int j = 0; j < NUM_MOTORS; j++)
    {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      desired_joint_positions[i][j] = atof(strtokIndx);
      received += String(desired_joint_positions[i][j], 4) + " ";
    }

    Serial.println(received);
  }

  position_trajectory = true;
  current_trajectory_point = 0;
  last_time = 0.0;
}

void ramp_tf()
{
  float tr;
  float tm;
  float tf[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  float delta_x;

  tf_max = 0;
  for (int idx = 0; idx < NUM_MOTORS; idx++)
  {
    delta_x = abs(desired_joint_positions[current_trajectory_point][idx] - joint_positions[idx]);
    tr = min(delta_x / vmax, vmax / amax);
    
    tm = max(0, delta_x / vmax - vmax / amax);
    tf[idx] = 2 * tr + tm;
    Serial.println("Ramp tf");
    Serial.print(idx);
    Serial.print("\t");
    Serial.print(tf[idx]);
    Serial.print("\t");
    Serial.print(tf_max);
    Serial.print(delta_x / vmax);
    Serial.print("\t");
    Serial.print(vmax / amax);
    Serial.print("\t");
    Serial.print("0\t");
    Serial.print(delta_x / vmax - vmax / amax);
    Serial.println();
    tf_max = max(tf_max, tf[idx]); 
  }
}

void ramp_pos(float t)
{
  // Modify qCmd with the x(t) values of a ramp traj
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    float delta_x = desired_joint_positions[current_trajectory_point][i] - joint_positions[i];
    if (delta_x < 0) {
      vmax = -joint_velocities[i];
      amax = -amax_ref;
    } else {
      vmax = joint_velocities[i];
      amax = amax_ref;
    }
    float tr = min(delta_x / vmax, vmax / amax);
    float tm = max(0, delta_x / vmax - vmax / amax);
    // Piece wise integration
    if (t < tr) {
      qCmd[i] = joint_positions[i] + 0.5 * amax * pow(t, 2);
    } else if (tr <= t & t < tr + tm) {
      qCmd[i] = joint_positions[i] + (0.5 * amax * pow(tr, 2)) + vmax * (t - tr);
    } else if (tr + tm <= t & t < 2 * tr + tm) {
      qCmd[i] = joint_positions[i] + (0.5 * amax * pow(tr, 2)) + (vmax * tm) + (vmax * (t - tm - tr) - 0.5 * amax * pow(t - tm - tr, 2));
    } else {
      qCmd[i] = desired_joint_positions[current_trajectory_point][i];
    }
  }
}

void ramp_vel(float t)
{
  // Modify q_dotCmd with the v(t) values of a ramp traj
  for (int i = 0; i < NUM_MOTORS; i++) {
    float delta_x = desired_joint_positions[current_trajectory_point][i] - joint_positions[i];
    float pure_PID_tol = 1; // If dx is small, ref commands may cause overshoot // 1 MM???
    if (delta_x < 0) {
      vmax = -joint_velocities[i];
      amax = -amax_ref;
    } else {
      vmax = joint_velocities[i];
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
    if (abs(delta_x) < pure_PID_tol) {
      q_dotCmd[i] = 0.0;
    }
  }
}

void cmdPos()
{
  for (int i = 0; i < NUM_MOTORS; i++) {
    _joint_positionsCmd[i] = qCmd[i];
  }
}

void cmdVel()
{
  for (int i = 0; i < NUM_MOTORS; i++) {
    _joint_velocitiesCmd[i] = q_dotCmd[i];
    _refCmd[i] = q_dotCmd[i] * VEL2CMD;
  }
}

void send_motorCmd()
{
  for (int i = 0; i < NUM_MOTORS; i++) {
    int motor_cmd_val = _motorCmd[i];
    motor_cmd_val = min(max(motor_cmd_val, -255), 255); // Clip to safe bounds
    if (motor_cmd_val > 0) { // 20 threshold?
      motors[i]->run(FORWARD); 
    } else if (motor_cmd_val <= 0) {
      motors[i]->run(BACKWARD);
    } else {
      motors[i]->run(RELEASE);
    }
    motors[i]->setSpeed((uint8_t)(abs(motor_cmd_val)));
  }
}

void driveController()
{
  updateState();

  float PID_Cmd[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  for (int i = 0; i < NUM_MOTORS; i++) {
    float error = _joint_positionsCmd[i] - joint_positions[i];
    float p_command = p * error;
    _motorCmd[i] = (int)(_refCmd[i] + 0.5 * p_command); //TODO change 0.5?
  }
  send_motorCmd();
}

void ramp2pos()
{
  updateJointPositions();
  ramp_tf();

  float _traj_t0 = millis() / 1000.0;
  float _traj_tf = tf_max;
  float _traj_t = 0;

  while (_traj_t < _traj_tf)
  {
    Serial.println("in while loop");
    _traj_t = (millis() / 1000.0) - _traj_t0;

    ramp_pos(_traj_t);
    ramp_vel(_traj_t);

    cmdPos();
    cmdVel();
    driveController();

    float q = joint_positions[0];
    float qq = qCmd[0];
    float q_dot = joint_velocities[0];
    float qq_dot = q_dotCmd[0];

    Serial.println("RAMP2POS");
    Serial.print(_traj_t); Serial.print('\t');
    Serial.print(_traj_tf); Serial.print('\t');
    Serial.print("q joint positions\t");
    Serial.print(1000*q); Serial.print('\t');
    Serial.print("qq joint pos cmd\t");
    Serial.print(1000*qq); Serial.print('\t');
    Serial.print("qdot joint velocities\t");
    Serial.print(1000*q_dot); Serial.print('\t');
    Serial.print("qqdot joint vel cmd\t");
    Serial.println(1000*qq_dot);

//    break;
    //
    //    delay(50);
  }
  Serial.println(_traj_t);
  Serial.println(_traj_tf);
  Serial.println("Exit while loop");
}

void executeWaypoints()
{
  bool reached_point = true;
  Serial.print("E, ");
  for (int idx = 0; idx < NUM_MOTORS; idx++)
  {
    if (motors[idx] != NULL)
    {
      updateState();
      joint_errors[idx] = joint_positions[idx] - desired_joint_positions[current_trajectory_point][idx];
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
  Serial.println("Checked errors");
  Serial.println(reached_point);
  Serial.println();
  // If all motors have reached their desired position, move onto the next trajectory
  if (reached_point)
  {
    Serial.println("reached point, onto next trajectory");
    current_trajectory_point += 1;
  }
  
  if (current_trajectory_point < num_trajectory_points)
  {
    ramp2pos();
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

void moveDeltaPosition()
{
  bool reached_point = true;
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
      } else
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

  // P control
  float time_left = max(durations[current_trajectory_point] - last_time, 0.1);
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
          float pid = p * joint_errors[i];
          int motor_speed = (int)(min(max(0.0, pid), 1.0) * 255.0); //change it to use duration input
          // Serial.print(motor_speed);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(FORWARD);
        }
        // Move backwards
        else if (joint_errors[i] < -position_threshold)
        {
          //  Serial.println("Backwarding...");
          float pid = p * joint_errors[i];
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

void publishJointStates()
{
  joint_states = "j,";

  for (int i = 0; i < NUM_MOTORS; i++)
  {
    joint_states += String(joint_positions[i], 4) + ","; //+ String(joint_velocities[i],4) + ",";
  }
  Serial.println(joint_states);
}

// SETUP CODE
void setup()
{
  Serial.begin(57600);
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
  joint_states.reserve(200); // Reserve 200 Bytes for joint_states
  resetJoints();
  Serial.println("SETUP MOVING");
//  moveDeltaPosition();
  executeWaypoints();   
  Serial.println("SETUP FINISHED");
}

// LOOP CODE
void loop()
{
  // Mode selection
  recvWithStartEndMarkers();
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
        executeWaypoints();   
    }
  }

  // Update state
  updateState();

  // Print joint states
  publishJointStates();

  // Pause prior to looping again
  //    delay(50);
}
