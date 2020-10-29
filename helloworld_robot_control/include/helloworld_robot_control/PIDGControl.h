#ifndef PROJECT_PIDGCONTROL_H
#define PROJECT_PIDGCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <helloworld_robot_control/ControlDataPublisher.h>
#include <helloworld_robot_control/hasRunning.h>
#include <helloworld_robot_control/setTrajectoryGoal.h>


namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        class PIDGControl : public ControlEffort, public ControlDataPublisher, public hasRunning {

        private:

            bool m_startFlag;//indicates that the update function has been run at least one within the current run
            bool running;//indicates that the controller is running

            JointState m_qPark;
            JointState m_qHome;
            JointState m_qInit;

            Matrix3d m_Kp;
            Matrix6d m_Kd;
            Matrix3d m_Ki;
            Matrix6d N_Kp;
            Matrix6d N_Kd;
            Matrix4d T0_W;
            Matrix4d TW_0;
            Matrix6d Eye;

            Vector6d m_goal;
            Vector6d m_xStart;
            Vector6d length_vec;
            Vector3d error;
            Vector6d nextGoal;
            Vector4d endstop1;
            Vector4d endstop2;
            double m_totalTime;
            double oldTime;
            double nextDuration;
            bool nextIdentification;
            double currentPathBeginTime;

            ros::ServiceServer goalSetterService;


        public:

            PIDGControl(double weight = 1.0, const QString &name = "PIDControl");

            ~PIDGControl();

            void setQInit(const JointState &qinit);
            void setQHome(const JointState &qhome);
            void setQPark(const JointState &qpark);
            bool isRunning() override;

        private:
            bool init();
            bool start();
            bool stop();
            bool setNewTrajectory(Vector6d goal, double trajectoryDuration, bool identification);
            bool setNewTrajectoryCallback(helloworld_robot_control::setTrajectoryGoal::Request &req, helloworld_robot_control::setTrajectoryGoal::Response &resp);


            Vector6d update(const RobotTime &time, const JointState &current);

            Eigen::Matrix<double, 6, 6> Jacobian(Vector6d q, Vector6d l);
            Eigen::Matrix<double, 4, 4> Transform_EF_W(Vector6d q, Vector6d l);


        };


    }
}
#endif //PROJECT_PDGCONTROL_H
