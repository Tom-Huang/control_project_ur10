//
// Created by hcg-ubuntu on 19-3-2.
//
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"

#include <iostream>
#include <sstream>
#include <fstream>

#include "ros/ros.h"
#include <sstream>

#include <cv_tracker/destination_msg.h>

using namespace std;

void chatterCallback(const cv_tracker::destination_msg& msg)
{
    ROS_INFO_STREAM("Frame No. " << msg.frame_index);
    ROS_INFO_STREAM("The destination position is: ["
                    << msg.x << ","
                    << msg.y << ","
                    << msg.z << "]"
                    <<"Quaternion: [rx, ry, rz, w]: [" << msg.rx << ","
                                                 << msg.ry << ","
                                                 << msg.rz << "]\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "destination_subscriber");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("helloworld/ef_destination", 1000, chatterCallback);

    ros::spin();

    return 0;
}

