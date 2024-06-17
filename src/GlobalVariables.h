#ifndef GLOBAL_VARIABLES_H
#define GLOBAL_VARIABLES_H

#define DIR_Y_PIN           4 // Direction #33
#define STEP_Y_PIN          2 // Step #32
#define ENABLE_Y_PIN        15 // Enable #27
#define STALLGUARD_Y_PIN    34 // Stallguard Interrupt Pin #12
#define DRIVER_ADDRESS_Y    0b00 // TMC2209 Driver address #0b01

#define DIR_X_PIN           33 // Direction #4
#define STEP_X_PIN          32 // Step #2
#define ENABLE_X_PIN        27 // Enable #15
#define STALLGUARD_X_PIN    12 // Stallguard Interrupt Pin #34
#define DRIVER_ADDRESS_X    0b01 // TMC2209 Driver address #0b00

#define R_SENSE       0.11f // 0.11 for MKS TMC2209

#define RXD2 26
#define TXD2 25
const long SERIAL_BAUD_RATE = 115200;

extern bool motor_enable;
extern bool home_check;

extern int stallguard_value_y;
extern int stallguard_value_x;
extern int rmscurrent;
extern int accel;
extern int max_speed;
extern int motor_microsteps;
extern int offset;

extern bool sd1_conncted;
extern bool sd2_conncted;

extern int current_position[2];
extern int wanted_position[2];
extern int x_max;
extern int x_min;
extern int y_max;
extern int y_min;

extern SemaphoreHandle_t uiMutex;

void UI_TakeMutex();
void UI_GiveMutex();


#endif