#include <iostream>

#include "ros/ros.h"
#include "leap_motion/Human.h"
#include "leap_motion/Connect.h"
#include "../inc/lmc_listener.h"
#include "Leap.h"

using namespace Leap;

// int main(int argc, char** argv) 
// {
//     int count=0;
//     ros::init(argc, argv, "leap_motion");
//     ros::NodeHandle nh1("leap_motion");

//     bool setup_params[7];

//     // Read parameters from the defined in listener_params.yaml
//     nh1.getParam("/enable_controller_info", setup_params[0] );
//     nh1.getParam("/enable_frame_info", setup_params[1] );
//     nh1.getParam("/enable_hand_info", setup_params[2] );

//     nh1.getParam("/enable_gesture_circle", setup_params[3] );
//     nh1.getParam("/enable_gesture_swipe", setup_params[4] );
//     nh1.getParam("/enable_gesture_screen_tap", setup_params[5] );
//     nh1.getParam("/enable_gesture_key_tap", setup_params[6] );

//     LeapListener listener(setup_params);
//     //ros::Publisher lm2ros_pub;
//     // Add a publisher to the leapListener object
//     listener.ros_publisher = nh1.advertise<leap_motion::Human>("leap_device", 1);
//     //lm2ros_pub = nh1.advertise<leap_motion::Connect>("helloworld/lm2ros_msg", 100);
//     Controller controller;
//     controller.addListener(listener);
//     // Keep doing ROS spin until shutdown() or Ctrl+C
 
//     ros::spin();
//     controller.removeListener(listener);

//     return 0;
// }

int main(int argc, char** argv) 
{   
   
    ros::init(argc, argv, "leap_motion");
    ros::NodeHandle nh1("leap_motion");
    bool setup_params[7];
    ros::NodeHandle nh2; // 声明一个节点句柄来与ROS系统进行通信
            //ROS_INFO("23333");
    ros::Publisher lm2ros_pub = nh2.advertise<leap_motion::Connect>("helloworld/lm2ros_msg", 1000);

    
    // Read parameters from the defined in listener_params.yaml
    nh1.getParam("/enable_controller_info", setup_params[0] );
    nh1.getParam("/enable_frame_info", setup_params[1] );
    nh1.getParam("/enable_hand_info", setup_params[2] );

    nh1.getParam("/enable_gesture_circle", setup_params[3] );
    nh1.getParam("/enable_gesture_swipe", setup_params[4] );
    nh1.getParam("/enable_gesture_screen_tap", setup_params[5] );
    nh1.getParam("/enable_gesture_key_tap", setup_params[6] );
    ros::Rate loop_rate(1);
    LeapListener listener(setup_params);
    
    //ros::Publisher lm2ros_pub;
    // Add a publisher to the leapListener object
    listener.ros_publisher = nh1.advertise<leap_motion::Human>("leap_device", 1);
    Controller controller;
  
  while (ros::ok())
  {
    //lm2ros_pub = nh1.advertise<leap_motion::Connect>("helloworld/lm2ros_msg", 100);
    controller.addListener(listener);

 
    ros::spinOnce();
    loop_rate.sleep();
   
  }
   controller.removeListener(listener);
  return 0;
}
