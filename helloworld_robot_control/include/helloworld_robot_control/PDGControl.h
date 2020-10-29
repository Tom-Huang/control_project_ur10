#ifndef PROJECT_PDGCONTROL_H
#define PROJECT_PDGCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <helloworld_robot_control/ControlDataPublisher.h>
#include <helloworld_robot_control/hasRunning.h>
#include <helloworld_robot_control/setTrajectoryGoal.h>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {


        /**
         * A simple PD joint torque controller moving the robot to a goal configuration.
         * Gains are initially set via rosparam /helloWorldControl/PDGControl
         * If identification is activated, gravity parameter updates will be published to /gravityParameter_updates
         */
        class PDGControl : public ControlEffort, public ControlDataPublisher, public hasRunning{

        //member Variables
        private:

            bool m_startFlag;//indicates that the update function has been run at least one within the current run
            bool running;//indicates that the controller is running
            bool identificationFlag;
            bool moving;

            Matrix6d m_Kp; // rosparam /gains_p
            Matrix6d m_Kd; //rosparam /gains_d
            Vector6d m_goal;
            Vector6d m_qStart;
            JointState m_qPark;
            JointState m_qHome;
            JointState m_qInit;

            double m_totalTime;

            Vector6d nextGoal;
            double nextDuration;
            bool nextIdentification;
            double currentPathBeginTime;

            Vector7d gravityParameters;

            ros::Publisher pubGravityParameterUpdates;
            ros::ServiceServer goalSetterService;

        //member functions
        public:


            PDGControl(const QString &name = "PDControl");
            ~PDGControl();

            void setQPark(const JointState &qpark);
            void setQInit(const JointState &qinit);
            void setQHome(const JointState &qhome);
            bool isRunning() override;

        private:
            bool init();
            bool start();
            bool stop();
            bool setNewTrajectory(Vector6d goal, double trajectoryDuration, bool identification);
            bool setNewTrajectoryCallback(helloworld_robot_control::setTrajectoryGoal::Request &req, helloworld_robot_control::setTrajectoryGoal::Response &resp);

            //since this is torque control, update returns joint torques
            Vector6d update(const RobotTime &time, const JointState &current);

        };


    }
}
#endif //PROJECT_PDGCONTROL_H
