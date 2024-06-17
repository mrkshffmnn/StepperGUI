#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <FastAccelStepper.h>

void setZero(int offset);
void move_motors();
void move_delta(int motor, int steps);
void update_position();

void update_com_status();

void stallguard_changed(int stallguard_value);
void update_sg_results();

#endif // MOTORCONTROL_H