#ifndef SETUPFUNCTIONS_H
#define SETUPFUNCTIONS_H

#include <FastAccelStepper.h>
#include <TMCStepper.h>

extern TMC2209Stepper driver_x;
extern TMC2209Stepper driver_y;
extern FastAccelStepper *stepper_x;
extern FastAccelStepper *stepper_y;

extern bool stalled_motor_x;
extern bool stalled_motor_y;

void setup_motors();
void setup_driver_x();
void setup_driver_y();
void check_driver_com();

#endif // SETUPFUNCTIONS_H