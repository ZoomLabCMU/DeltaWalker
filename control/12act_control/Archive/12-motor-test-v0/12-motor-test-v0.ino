#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>

#define NUM_MOTORS 12

// Create the three motor shield object I2C address input
Adafruit_MotorShield MC1 = Adafruit_MotorShield(0x60); // MC0 0x61 // MC2 0x62
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
                                        MC3_M1, MC3_M2, MC3_M3, MC3_M4};

// create object to access off-board analog-to-digital converter
Adafruit_ADS1015 ADC1; //ADC1 // ADC2
Adafruit_ADS1015 ADC2;
Adafruit_ADS1015 ADC3;

Adafruit_ADS1015* adcs[NUM_MOTORS] = {&ADC1, &ADC1, &ADC1, &ADC1,
                                      &ADC2, &ADC2, &ADC2, &ADC2,
                                      &ADC3, &ADC3, &ADC3, &ADC3,};
int channels[NUM_MOTORS] = {0,1,2,3,
                            0,1,2,3,
                            0,1,2,3};

double joint_positions[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0};

int motor_val[NUM_MOTORS] = {0, 0, 0, 0,
                             0, 0, 0, 0,
                             0, 0, 0, 0};


void updateJointPositions()
{
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    if(motors[i] != NULL){
        motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]);
        joint_positions[i] = motor_val[i] * 20.0 / 1650; // 1 bit = 2mV @ 4.096 FSR 0.02 mm stroke 3.3V range //* 0.0000303; // 100mm / 1650 analog reading
    }
  }
}

// SETUP CODE
void setup()
{
  // set all the base dc motor control pins to outputs
  Serial.begin(9600);           //  setup serial
  Serial.println("Start!");
  MC1.begin();
  MC2.begin();
  MC3.begin();
  Serial.println("Start MC!");
  // start all the ADCs
  ADC1.begin(0x48); // ADC1: 0x49 // ADC2: 0x4B
  ADC2.begin(0x49);
  ADC3.begin(0x4A);
  Serial.println("Start ADC!");
  ADC1.setGain(GAIN_ONE); // 4.096V FSR
  ADC2.setGain(GAIN_ONE);
  ADC3.setGain(GAIN_ONE);

  // disable dc motors by setting their enable lines to low
  for(int i=0; i<NUM_MOTORS; i++){
    motors[i]->setSpeed(0);
    motors[i]->run(RELEASE);
  }
  
  Serial.println("Start!");
  // Serial.println(ADC1.getGain());
  // Serial.println(ADC1.getDataRate()); 
}


// LOOP CODE
void loop()
{
  Serial.println("Test!");
  for (int m = 0; m < 12; m++){
    Serial.print("# "); Serial.print(m); Serial.println(" ");
    for(int i = 0; i < 10; i++){
      motors[m]->setSpeed(100);
      motors[m]->run(FORWARD);
      delay(50);
    }
    motor_val[m] = adcs[m]->readADC_SingleEnded(channels[m]);
    joint_positions[m] = motor_val[m] * 3.0 * 0.02 / 5.0; //* 20.0 / 1650;
    
    Serial.print(joint_positions[m]); Serial.print(" ");
    Serial.print(motor_val[m]); Serial.println("");
    delay(50);

    Serial.print("# "); Serial.print(m); Serial.println(" ");
    for(int i = 0; i < 10; i++){
      motors[m]->setSpeed(100);
      motors[m]->run(BACKWARD);
      delay(50);
    }
    motor_val[m] = adcs[m]->readADC_SingleEnded(channels[m]);
    joint_positions[m] = motor_val[m] * 3.0 * 0.02 / 5.0; //* 20.0 / 1650;
    Serial.print(joint_positions[m]); Serial.print(" ");
    Serial.print(motor_val[m]); Serial.println("");
    delay(50);

    motors[m]->setSpeed(0);
    motors[m]->run(RELEASE);
    delay(50);
  }
}

