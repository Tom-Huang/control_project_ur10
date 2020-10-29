//
// Created by helloworld on 02.03.19.
//
#ifndef PROJECT_CONTROLDATAPUBLISHER_H
#define PROJECT_CONTROLDATAPUBLISHER_H
#include<tumtools/Math/EigenDefs.h>
#include <ros/ros.h>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {


        class ControlDataPublisher {

            public:

            protected:
                ros::NodeHandle n;
                ros::Publisher pubCtrlData;
                void publish(const double &t, const Tum::VectorDOFd &q, const Tum::VectorDOFd &qp, const Tum::VectorDOFd &qpp, const Tum::VectorDOFd &qd, const Tum::VectorDOFd &qpd, const Tum::VectorDOFd &deltaQ, const Tum::VectorDOFd &deltaQp, const Tum::VectorDOFd &torques);

        };
    }
}


#endif //PROJECT_CONTROLDATAPUBLISHER_H
