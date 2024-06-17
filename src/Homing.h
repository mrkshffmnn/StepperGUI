#ifndef HOMING_H
#define HOMING_H

extern int offset;

void home_axis();
void home_y();
void home_x();
void homing_y();
void homing_x();
void reset_stepper_y();
void reset_stepper_x();


#endif // HOMING_H