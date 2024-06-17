#include "MotorControl.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#include "ui/ui.h"
#include <lvgl.h>
#include "GlobalVariables.h"
#include "SetupFunctions.h"


void update_com_status(){
  UI_TakeMutex();
  if (sd1_conncted){
    lv_obj_add_state(ui_sd1checkbox, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(ui_sd1checkbox, LV_STATE_CHECKED);
  }

  if (sd2_conncted){
    lv_obj_add_state(ui_sd2checkbox, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(ui_sd2checkbox, LV_STATE_CHECKED);
  }
  UI_GiveMutex();
}

int current_position[2]= {0,0};

int x_max = 1650;
int x_min = 0;
int y_max = 1650;
int y_min = 0;

void setZero(int offset){
  current_position[0] = offset;
  current_position[1] = offset;
  stepper_x->setCurrentPosition(offset);
  stepper_y->setCurrentPosition(offset);

  Serial.println();
  Serial.print("Zeroed:");
  Serial.print(current_position[0]);
  Serial.print("/");
  Serial.print(current_position[1]);
}

void move_motors(){
  if (motor_enable){

    stepper_x->setAcceleration(accel);                                
    stepper_x->setSpeedInHz(max_speed);

    stepper_y->setAcceleration(accel);                                
    stepper_y->setSpeedInHz(max_speed);

    stepper_x->moveTo(wanted_position[0], false);
    stepper_y->moveTo(wanted_position[1], false);

  } else {
    Serial.print("Motor Disabled.");
  }
}

void move_delta(int motor, int steps) {
    if (motor_enable) {
        int new_target_position = 0;
        int current_pos = (motor == 0) ? current_position[0] : current_position[1];
        int target_pos = (motor == 0) ? wanted_position[0] : wanted_position[1];

        // Calculate new target position
        if ((current_pos + steps) <= ((motor == 0) ? x_max : y_max) && (current_pos + steps) >= ((motor == 0) ? x_min : y_min)) {
            new_target_position = current_pos + steps;
        } else {
            // Adjust target position to stay within limits
            if (steps > 0) {
                new_target_position = (motor == 0) ? x_max : y_max;
            } else {
                new_target_position = (motor == 0) ? x_min : y_min;
            }
        }

        // Update target position based on motor
        if (motor == 0) {
            wanted_position[0] = new_target_position;
        } else {
            wanted_position[1] = new_target_position;
        }

        // Perform the movement
        if (motor == 0) {
            stepper_x->setAcceleration(accel);
            stepper_x->setSpeedInHz(max_speed);
            stepper_x->moveTo(wanted_position[0], false);
        } else {
            stepper_y->setAcceleration(accel);
            stepper_y->setSpeedInHz(max_speed);
            stepper_y->moveTo(wanted_position[1], false);
        }
    } else {
        Serial.println("Motor Disabled.");
    }
}

void update_position(){

    // Update wanted position label
    char wanted_pos_str[16]; // Assuming the max length of the string representation of the integer is 16
    snprintf(wanted_pos_str, sizeof(wanted_pos_str), "%d, %d", wanted_position[0], wanted_position[1]);
    
    // Update current position label
    char current_pos_str[16]; // Assuming the max length of the string representation of the integer is 16
    snprintf(current_pos_str, sizeof(current_pos_str), "%d, %d", current_position[0], current_position[1]);
    
    UI_TakeMutex();
    lv_label_set_text(ui_wantedpositionlabel, wanted_pos_str);
    lv_label_set_text(ui_actualpositionlabel, current_pos_str);
    UI_GiveMutex();
    
}

int sg_results_x = 0;
int sg_results_y = 0;

void update_sg_results(){
    sg_results_x = driver_x.SG_RESULT()/2;
    sg_results_y = driver_y.SG_RESULT()/2;

    char sg_x_str[16]; // Assuming the max length of the string representation of the integer is 16
    snprintf(sg_x_str, sizeof(sg_x_str),  "%d", sg_results_x);
    
    char sg_y_str[16]; // Assuming the max length of the string representation of the integer is 16
    snprintf(sg_y_str, sizeof(sg_y_str),  "%d", sg_results_y);

    UI_TakeMutex();
    lv_label_set_text(ui_sgresultx, sg_x_str);
    lv_label_set_text(ui_sgresulty, sg_y_str);

    // Assuming stalled_motor_x and stalled_motor_y are boolean variables indicating motor stall
     if (stalled_motor_x){
        lv_obj_add_state(ui_sgresultxcheckbox, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(ui_sgresultxcheckbox, LV_STATE_CHECKED);
    }

    if (stalled_motor_y){
        lv_obj_add_state(ui_sgresultycheckbox, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(ui_sgresultycheckbox, LV_STATE_CHECKED);
    } 

    UI_GiveMutex();
}