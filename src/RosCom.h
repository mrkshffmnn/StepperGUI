/* #ifndef ROSCOM_H
#define ROSCOM_H



#ifdef __cplusplus
extern "C" {
#endif */
#include <std_msgs/Int32MultiArray.h>
void start_ros_com();
void receive_position(const std_msgs::Int32MultiArray& cmd_msg);
void publish_position();

/* #ifdef __cplusplus
} /* extern "C" */
//#endif

//#endif // ROSCOM_H */