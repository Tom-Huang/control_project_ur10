#include <helloworld_robot_control/ControlDataPublisher.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <iostream>


namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        void ControlDataPublisher::publish(const double &t, const Tum::VectorDOFd &q, const Tum::VectorDOFd &qp, const Tum::VectorDOFd &qpp,
                                      const Tum::VectorDOFd &qd, const Tum::VectorDOFd &qpd, const Tum::VectorDOFd &deltaQ,
                                      const Tum::VectorDOFd &deltaQp, const Tum::VectorDOFd &torques) {

            tum_ics_ur_robot_msgs::ControlData msg;

            msg.header.stamp = ros::Time::now();

            msg.time = t;

            for (int i = 0; i < STD_DOF; i++) {
                msg.q[i] = q(i);
                msg.qp[i] = qp(i);
                msg.qpp[i] = qpp(i);

                msg.qd[i] = qd(i);
                msg.qpd[i] = qpd(i);

                msg.Dq[i] = deltaQ(i);
                msg.Dqp[i] = deltaQp(i);

                msg.torques[i] = torques(i);
            }

            pubCtrlData.publish(msg);
        }
    }
}
