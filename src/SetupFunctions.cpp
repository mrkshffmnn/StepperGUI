#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#include "GlobalVariables.h"
#include "SetupFunctions.h"

HardwareSerial TMCSerial(2);

TMC2209Stepper driver_x(&TMCSerial, R_SENSE, DRIVER_ADDRESS_X);
TMC2209Stepper driver_y(&TMCSerial, R_SENSE, DRIVER_ADDRESS_Y);
using namespace TMC2209_n;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper_x = NULL;
FastAccelStepper *stepper_y = NULL;

int rmscurrent        = 400;
int accel             = 2000;
int max_speed         = 500;
int motor_microsteps  = 0;
int stallguard_value_y = 45;
int stallguard_value_x = 60;

//----------------------------------------------------------
bool stalled_motor_x = false;
void IRAM_ATTR stalled_position_x(){stalled_motor_x = true;}

bool  stalled_motor_y = false;
void IRAM_ATTR stalled_position_y(){stalled_motor_y = true;}
//----------------------------------------------------------
void setup_tmc_serial(){
  TMCSerial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, RXD2, TXD2);           // INITIALIZE SERIAL2 UART FOR TMC2209
  while(!TMCSerial);
  Serial.println("TMCSerial Started"); 
}
//----------------------------------------------------------
void setup_driver_y(){
  pinMode(DIR_Y_PIN, OUTPUT);
  pinMode(STEP_Y_PIN, OUTPUT);
  pinMode(ENABLE_Y_PIN, OUTPUT);
  pinMode(STALLGUARD_Y_PIN , INPUT);
  digitalWrite(ENABLE_Y_PIN, LOW);        // Enable driver in hardware

  driver_y.begin();                       // Start all the UART communications functions behind the scenes
  driver_y.toff(4);                       // For operation with StealthChop, this parameter is not used, but it is required to enable the motor. In case of operation with StealthChop only, any setting is OK
  driver_y.senddelay(2);
  driver_y.pdn_disable(1);
  driver_y.microsteps(0);
  driver_y.rms_current(rmscurrent);       // Set the current in milliamps.
  driver_y.I_scale_analog(false);         // Disabled to use the extrenal current sense resistors 
  driver_y.internal_Rsense(false);        // Use the external Current Sense Resistors.
  driver_y.TCOOLTHRS(0xFFFFF);             //tcolls Minimum speed at which point to turn on StallGuard. StallGuard does not work as very low speeds such as the beginning of acceleration so we need to keep it off until it reaches a reliable speed.
  driver_y.SGTHRS(stallguard_value_y);      // Set the stall value from 0-255. Higher value will make it stall quicker.  // Set the number of microsteps. Due to the "MicroPlyer" feature, all steps get converterd to 256 microsteps automatically. However, setting a higher step count allows you to more accurately more the motor exactly where you want.
  driver_y.shaft(false);                   // Set the shaft direction. 

  stepper_y = engine.stepperConnectToPin(STEP_Y_PIN);
  stepper_y->setDirectionPin(DIR_Y_PIN);
  stepper_y->setAcceleration(accel);                                  
  stepper_y->setSpeedInHz(max_speed);
  stepper_y->setCurrentPosition(0);
  attachInterrupt(digitalPinToInterrupt(STALLGUARD_Y_PIN), stalled_position_y, RISING);
}

void setup_driver_x(){
  pinMode(DIR_X_PIN, OUTPUT);
  pinMode(STEP_X_PIN, OUTPUT);
  pinMode(ENABLE_X_PIN, OUTPUT);
  pinMode(STALLGUARD_X_PIN , INPUT);
  digitalWrite(ENABLE_X_PIN, LOW);        // Enable driver in hardware

  driver_x.begin();                       // Start all the UART communications functions behind the scenes
  driver_x.toff(6);                       // For operation with StealthChop, this parameter is not used, but it is required to enable the motor. In case of operation with StealthChop only, any setting is OK
  driver_x.senddelay(4);
  driver_y.pdn_disable(1);
  driver_x.rms_current(rmscurrent);       // Set the current in milliamps.
  driver_x.microsteps(0);
  driver_x.I_scale_analog(false);         // Disabled to use the extrenal current sense resistors 
  driver_x.internal_Rsense(false);        // Use the external Current Sense Resistors.
  driver_x.TCOOLTHRS(0xFFFFF);             //tcolls Minimum speed at which point to turn on StallGuard. StallGuard does not work as very low speeds such as the beginning of acceleration so we need to keep it off until it reaches a reliable speed.
  driver_x.SGTHRS(stallguard_value_x);      // Set the stall value from 0-255. Higher value will make it stall quicker.  // Set the number of microsteps. Due to the "MicroPlyer" feature, all steps get converterd to 256 microsteps automatically. However, setting a higher step count allows you to more accurately more the motor exactly where you want.
  driver_x.shaft(false);                   // Set the shaft direction. 

  stepper_x = engine.stepperConnectToPin(STEP_X_PIN);
  stepper_x->setDirectionPin(DIR_X_PIN);
  stepper_x->setAcceleration(accel);                                
  stepper_x->setSpeedInHz(max_speed);
  stepper_x->setCurrentPosition(0);

  attachInterrupt(digitalPinToInterrupt(STALLGUARD_X_PIN), stalled_position_x, RISING);
}
//----------------------------------------------------------

void setup_motors() {
  //start hardwareserial for tmc drivers
  setup_tmc_serial();

  //Initialize FastAccelStepper
  engine.init();

  //setup drivers
  setup_driver_y();
  setup_driver_x();

  check_driver_com();
}

bool sd1_conncted = false;
bool sd2_conncted = false;

void check_driver_com(){
  driver_x.microsteps(2);
  delay(50);
  if (driver_x.microsteps() == 2)
  {
    sd1_conncted = true;
    Serial.println("Stepper Driver 1 CONNECTED");
  }else{
    sd1_conncted = false;
    Serial.println("Stepper Driver 1 NOT CONNECTED");
  }
  delay(50);

  driver_y.microsteps(2);
  delay(50);
  if (driver_y.microsteps() == 2)
  {
    sd2_conncted = true;
    Serial.println("Stepper Driver 2 CONNECTED");
  }else{
    sd2_conncted = false;
    Serial.println("Stepper Driver 2 NOT CONNECTED");
  }

  driver_x.microsteps(motor_microsteps);
  driver_y.microsteps(motor_microsteps);
}