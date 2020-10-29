#include "ros/ros.h"
#include <tf/transform_datatypes.h>

#include "leap_motion/Human.h"
#include "leap_motion/Hand.h"
#include "leap_motion/Finger.h"
#include "leap_motion/Bone.h"
#include "leap_motion/Gesture.h"
#include "leap_motion/Connect.h"

#include <iostream>

#include "../inc/lmc_listener.h"
#include "Leap.h"

using namespace Leap;

void msgCallback(const leap_motion::Connect::ConstPtr& msg)
{

ROS_INFO("Mode:%d Gripper:%d Position:(%d %d %d) Revolution(%d %d %d) 2hands: %d",
 msg->is_translation, msg->is_grab, msg->is_forward, msg->is_right, msg->is_up, msg->is_roll, msg->is_pitch, msg->is_yaw, msg->is_twohands); // 
}

int main(int argc, char **argv) // 节点主函数
{
ros::init(argc, argv, "lm2ros_sub"); // 初始化节点名称
ros::NodeHandle nh3; // 声明用于ROS系统和通信的节点句柄
// 声明订阅者，创建一个订阅者ros_tutorial_sub，
// 它利用ros_tutorials_topic功能包的的MsgTutorial消息文件。
// 话题名称是"ros_tutorial_msg"，订阅者队列（queue)的大小设为100。
ros::Subscriber lm2ros_sub = nh3.subscribe("helloworld/lm2ros_msg", 1000, msgCallback);
// 用于调用后台函数，等待接收消息。在接收到消息时执行后台函数。
ros::spin();
return 0;
}