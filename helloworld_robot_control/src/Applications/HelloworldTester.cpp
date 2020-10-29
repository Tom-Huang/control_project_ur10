#include<tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include <helloworld_robot_control/TorqueFusionController.h>
#include<helloworld_robot_control/PDGControl.h>
#include <helloworld_robot_control/PIDGControl.h>
#include <helloworld_robot_control/LeapControl.h>
#include <helloworld_robot_control/OrientationControl.h>
#include <helloworld_robot_control/GravityControl.h>
#include <helloworld_robot_control/OrientationLeapControl.h>
#include<QApplication>
#include<visualization_msgs/Marker.h>

using namespace Eigen;

int main(int argc, char **argv)
{

    QApplication a(argc,argv);

    ros::init(argc,argv,"helloworldRobotArm",ros::init_options::AnonymousName);

    std::cout << "Test" << std::endl;

    QString configFilePath=argv[1];

//    ROS_INFO_STREAM("Config File: "<<configFilePath.toStdString().c_str());

    tum_ics_ur_robot_lli::Robot::RobotArmConstrained robot(configFilePath);


    //starts robotArm communication and the thread
    //inits Kinematic Model, Dynamic Model
    if(!robot.init())
    {
        return -1;
    }

    tum_ics_ur_robot_lli::RobotControllers::TorqueFusionController torqueFusionController;
    torqueFusionController.setQHome(robot.qHome());
    torqueFusionController.setQPark(robot.qPark());

    tum_ics_ur_robot_lli::RobotControllers::PDGControl pdgControl;
    tum_ics_ur_robot_lli::RobotControllers::PIDGControl pidgControl;
    tum_ics_ur_robot_lli::RobotControllers::LeapControl leapControl;
    tum_ics_ur_robot_lli::RobotControllers::OrientationControl orientationControl;
    tum_ics_ur_robot_lli::RobotControllers::OrientationLeapControl orientationLeapControl;
    tum_ics_ur_robot_lli::RobotControllers::GravityControl gravityControl;

    torqueFusionController.add(&pdgControl);
    torqueFusionController.add(&pidgControl);
    torqueFusionController.add(&leapControl);
    torqueFusionController.add(&orientationControl);
    torqueFusionController.add(&orientationLeapControl);
    torqueFusionController.add(&gravityControl);





    //The control must be connected to the robot after the init()-->The dynamic model needs to
    //be initialized!
    //also calls control.init(), e.g. load ctrl gains
    if(!robot.add(&torqueFusionController))
    {
        return -1;
    }




    robot.start();

    ROS_INFO_STREAM("Start main thread");

//    ros::Rate r(30);

//    while((ros::ok())&&(robot.isRunning()))
//    {
//        ros::spinOnce();
//        r.sleep();
//    }

    ros::spin();

    ROS_INFO_STREAM("main: Stoping RobotArm()");
    robot.stop(); //stops robotArm thread




    ROS_INFO_STREAM("main: Stopped!!!");

}








//    QString scriptFilePath="/home/dean/indigo_ws_iros_flo/src/tom_robot/tom_configs/Arms/parameters/Scripts/progTOM_right";

//    ur10_robot_lli::CommInterface commInterface(computerIp, commandPort, trajPort,robotIp, scriptFilePath, robotScriptPort);


//    usleep(20*1E6);


//    //commInterface.stop();

//    commInterface.setFinishMState();


//    ur10_robot_lli::ScriptLoader scriptLoader(robotIp,
//    "/home/dean/indigo_ws_iros_flo/src/tom_robot/tom_configs/Arms/parameters/Scripts/progParkTOM_RA");
