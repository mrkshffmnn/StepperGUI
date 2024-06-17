#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h> 

#include "MotorControl.h"
#include "ui/ui.h"
#include "RosCom.h"
#include "GlobalVariables.h"
#include "SetupFunctions.h"
#include "Homing.h"

#define LGFX_AUTODETECT // Autodetect board
#define LGFX_USE_V1     // set to use new version of library

#include <LovyanGFX.hpp> // main library
static LGFX lcd; // declare display variable

#include <lvgl.h>
#include "lv_conf.h"
/*** Setup screen resolution for LVGL ***/
static const uint16_t screenWidth = 480;
static const uint16_t screenHeight = 320;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

/*** Function declaration ***/
void display_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void init_display(){
  Serial.println();
  Serial.println("Display getting initilaized");
  lcd.init();
  lv_init();

  // Setting display to landscape
  if (lcd.width() < lcd.height())
    lcd.setRotation(lcd.getRotation() ^ 1);

  /* LVGL : Setting up buffer to use for display */
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

  /*** LVGL : Setup & Initialize the display device driver ***/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = display_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*** LVGL : Setup & Initialize the input device driver ***/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  lv_indev_drv_register(&indev_drv);
  
  ui_init();
}

void ROSSerialTask(void *pvParameters) {

    //start_ros_com(); // Initialize ROS communication

    for (;;) {

        // Publish current positions
        //publish_position();

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

SemaphoreHandle_t uiMutex;    // Semaphore handle for UI synchronization

void UI_TakeMutex() {
    // Take the UI mutex
    while (xSemaphoreTake(uiMutex, portMAX_DELAY) != pdPASS) {
        // Retry until successfully obtained
    }
}

void UI_GiveMutex() {
    // Give back the UI mutex
    xSemaphoreGive(uiMutex);
}

void TaskLVGL(void *pvParameters) {
    // Task to handle LVGL UI
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for LVGL handling

        UI_TakeMutex(); // Take the UI mutex before handling LVGL
        lv_task_handler(); // Handle LVGL tasks
        UI_GiveMutex(); // Give back the UI mutex
    }
}

void UpdateTask(void *pvParameters) {

    for (;;) {

      update_sg_results();
      update_position();

      vTaskDelay(pdMS_TO_TICKS(10));
    }
}

SemaphoreHandle_t moveMutex; // Mutex to protect motor movement
void move_task(void *pvParameters) {
    for (;;) {
        // Check if the stepper motors are running and update the current position
        if (stepper_x->isRunning()) {
            current_position[0] = stepper_x->getCurrentPosition();
        }
        if (stepper_y->isRunning()) {
            current_position[1] = stepper_y->getCurrentPosition();
        } else {
          current_position[0] = stepper_x->getCurrentPosition();
          current_position[1] = stepper_y->getCurrentPosition();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void HomeTask(void *pvParameters) {
  for (;;) {
    if (home_check) {
      Serial.println("Homing Started.");
      //home y
      home_y();
      //home x
      home_x();
      //set offset as zero
      setZero(offset);
      //done
      home_check = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

   }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial Started");
  delay(1000);

  setup_motors();

  xTaskCreate(HomeTask, "HomeTask", 5000, NULL, 1, NULL);

  //home_check = true;
  while(home_check){delay(1);}

  init_display();

  // Initialize the semaphore
  uiMutex = xSemaphoreCreateMutex();
  moveMutex = xSemaphoreCreateMutex();

  // Create tasks
  xTaskCreate(TaskLVGL, "LVGL_Task", 5000, NULL, 1, NULL);
  xTaskCreate(move_task, "Move_Task", 5000, NULL, 2, NULL);
  xTaskCreate(UpdateTask, "SGTask", 5000, NULL, 1, NULL);
  xTaskCreate(ROSSerialTask, "ROSSerialTask", 5000, NULL, 1, NULL);
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(23, 0);
  ledcWrite(0, 255);

  update_com_status();
  
}

void loop() {
  vTaskDelete(NULL);
}

    /*** Display callback to flush the buffer to screen ***/
  void display_flush(lv_disp_drv_t * disp, const lv_area_t *area, lv_color_t *color_p)
  {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    lcd.startWrite();
    lcd.setAddrWindow(area->x1, area->y1, w, h);
    lcd.pushPixels((uint16_t *)&color_p->full, w * h, true);
    lcd.endWrite();

    lv_disp_flush_ready(disp);
  }

  /*** Touchpad callback to read the touchpad ***/
  void touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
  {
    uint16_t touchX, touchY;
    bool touched = lcd.getTouch(&touchX, &touchY);

    if (!touched)
    {
      data->state = LV_INDEV_STATE_REL;
    }
    else
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touchX;
      data->point.y = touchY;

      // Serial.printf("Touch (x,y): (%03d,%03d)\n",touchX,touchY );
    }
  }
