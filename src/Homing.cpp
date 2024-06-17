#include <Arduino.h>
#include "GlobalVariables.h"
#include "SetupFunctions.h"
#include "MotorControl.h"

void reset_stepper_y(){
  //reset driver
  digitalWrite(ENABLE_Y_PIN, HIGH);
  delayMicroseconds(500);
  digitalWrite(ENABLE_Y_PIN, LOW);
  //signal step to re-enable diag pin
  stepper_y->forwardStep(true);
  stepper_y->backwardStep(true);
  stalled_motor_y = false;
  delayMicroseconds(500);
}

void reset_stepper_x(){
  //reset driver
  digitalWrite(ENABLE_X_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(ENABLE_X_PIN, LOW);
  //signal step to re-enable diag pin
  stepper_x->forwardStep(true);
  stepper_x->backwardStep(true);
  stalled_motor_x = false;
  delayMicroseconds(100);
}

int homing_counter = 0;
int homing_counter_limit = 5;

void homing_y(int speed, int sgthrs) 
{ 
    homing_counter = 0; // Reset the counter before attempting homing
    stepper_y->setSpeedInHz(speed);
    driver_y.SGTHRS(sgthrs);
    while (homing_counter < homing_counter_limit) {
        //homing try
        if (digitalRead(STALLGUARD_Y_PIN) == LOW) {

            //start-up of driver to stabilize diag output
            stepper_y->runBackward();
            delay(50);

            //run stepper until stall interrupt
            while (true) {
                int sgyResult = driver_y.SG_RESULT();
                if (sgyResult != 0 && (sgyResult / 2) <= sgthrs) {
                    Serial.print("\tFinal SG:");
                    Serial.print(sgyResult/2);
                    stepper_y->forceStop();
                    Serial.println();
                    break;
                } else if (sgyResult == 0) {
                    Serial.println("faulty zero");
                    vTaskDelay(1);
                } else {
                    Serial.println(driver_y.SG_RESULT()/2, DEC);
                    vTaskDelay(1);
                }
            }
            break; // Exit the loop since homing was successful
        } else {
            // Increment the counter if homing failed
            homing_counter++;
            // Print a message indicating homing failed
            Serial.println("Homing failed. Retrying...");
        }
    }

    if (homing_counter >= homing_counter_limit) {
        Serial.println("Homing failed after maximum attempts.");
        ESP.restart();
    }
}

void homing_x(int speed, int sgthrs)
{ 
    homing_counter = 0; // Reset the counter before attempting homing
    stepper_x->setSpeedInHz(speed);
    driver_x.SGTHRS(sgthrs);
    while (homing_counter < homing_counter_limit) {
        //homing try
        if (digitalRead(STALLGUARD_X_PIN) == LOW) {

            //start-up of driver to stabilize diag output
            stepper_x->runBackward();
            delay(50);

            //run stepper until stall interrupt
            while (true) {
                int sgxResult = driver_x.SG_RESULT();
                if (sgxResult != 0 && (sgxResult / 2) <= sgthrs) {
                    Serial.print("\tFinal SG:");
                    Serial.print(sgxResult/2);
                    stepper_x->forceStop();
                    Serial.println();
                    break;
                } else if (sgxResult == 0) {
                    Serial.println("faulty zero");
                    vTaskDelay(1);
                } else {
                    Serial.println(driver_x.SG_RESULT()/2, DEC);
                    vTaskDelay(1);
                }
            }
            break; // Exit the loop since homing was successful
        } else {
            // Increment the counter if homing failed
            homing_counter++;
            // Print a message indicating homing failed
            Serial.println("Homing failed. Retrying...");
        }
    }

    if (homing_counter >= homing_counter_limit) {
        Serial.println("Homing failed after maximum attempts.");
        ESP.restart();
    }
}

int home_fast_y = 30;
int home_slow_y = 6;
int home_fast_X = 30;
int home_slow_x = 4;
int offset = 50;

void home_y(){                             
  //fast homing
  Serial.println("Fast Homing Y.");
  homing_y(80, home_fast_y);
  delay(100);
  //back off
  Serial.println("Backing Off Y.");
  stepper_y->move(100, true);
  delay(100);
  //slow Homing
  Serial.println("Slow Homing Y.");
  homing_y(20, home_slow_y);
  delay(100);
  //move to offset
  Serial.println("Moving Offset Y.");
  stepper_y->setSpeedInHz(max_speed);
  stepper_y->move(offset, true);
  delay(100);
  //done
}

void home_x(){                             
  //fast homing
  Serial.println("Fast Homing X.");
  homing_x(80, home_fast_X);
  delay(100);
  //back of
  Serial.println("Backing Off X.");
  stepper_x->move(100, true);
  delay(100);
  //slow Homing
  Serial.println("Slow Homing X.");
  homing_x(20, home_slow_x);
  delay(100);
  //move to offset
  Serial.println("Moving Offset X.");
  stepper_x->setSpeedInHz(max_speed);
  stepper_x->move(offset, true);
  delay(100);
  //done
}

void home_axis(){
  home_check = true;
}