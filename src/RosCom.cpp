#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

#include "MotorControl.h"
#include "ui/ui.h"
#include <lvgl.h>
#include "GlobalVariables.h"

bool ros_communication_working = false;

// Forward declaration of receive_position function
void receive_position(const std_msgs::Int32MultiArray& cmd_msg);

//ros node and publisher
std_msgs::Int32MultiArray actual_position_msg;

ros::Publisher pub_actualpos("actual_position", &actual_position_msg);
ros::Subscriber<std_msgs::Int32MultiArray> sub("wanted_position", receive_position);

ros::NodeHandle nh;

// Initialize ROS communication
void start_ros_com(){
    nh.initNode();
    if (nh.connected()){
    nh.subscribe(sub);
    nh.advertise(pub_actualpos);
    } else{
        Serial.println("nh.connected = false");
    }

}

int wanted_position[2]= {0,0};

// Receiving wanted positions
void receive_position(const std_msgs::Int32MultiArray& cmd_msg){
    if (nh.connected()){
        if (cmd_msg.data_length == 2) { // Assuming you're expecting a pair of positions
            wanted_position[0] = cmd_msg.data[0];
            wanted_position[1] = cmd_msg.data[1];
            move_motors();
        } else {
            Serial.println("Invalid data received for wanted positions.");
        }
    } else {
        lv_obj_clear_state(ui_roscomcheckbox, LV_STATE_CHECKED);
        Serial.println("ROSCOM NOT Working");
    }
}

// Publish current positions
void publish_position(){
    if (nh.connected()){
    actual_position_msg.data_length = 2; // Assuming you're publishing a pair of positions
    actual_position_msg.data[0] = current_position[0];
    actual_position_msg.data[1] = current_position[1];
    pub_actualpos.publish(&actual_position_msg);
    nh.spinOnce();


    lv_obj_add_state(ui_roscomcheckbox, LV_STATE_CHECKED);
    Serial.println("ROSCOM Working");
    } else {
        lv_obj_clear_state(ui_roscomcheckbox, LV_STATE_CHECKED);
        Serial.println("ROSCOM NOT Working");
    }
    
}


