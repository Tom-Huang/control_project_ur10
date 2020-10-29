#ifndef HELLOWORLD_GRAVITYCONTROL_H
#define HELLOWORLD_GRAVITYCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <helloworld_robot_control/ControlDataPublisher.h>
#include <helloworld_robot_control/hasRunning.h>
#include <std_msgs/Float64MultiArray.h>



namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        /**
         * A controller compensating for gravity
         * gravity parameters are initially set via rosparam /helloWorldControl/GravityControl/gravityParameters
         */
        class GravityControl: public ControlEffort, public ControlDataPublisher, public hasRunning{

            //Member Variables
        private:

            bool m_startFlag;//indicates that the update function has been run at least one within the current run
            bool running;//indicates that the controller is running
            bool identificationFlag;

            JointState m_qPark;
            JointState m_qHome;
            JointState m_qInit;

            Vector7d gravityParameters;

            ros::Subscriber subGravityParameterUpdates;

            //Member functions
        public:
            GravityControl(const QString &name = "GravityControl");
            ~GravityControl();

            /**
             * Gravity regressor
             * @param q configuration
             * @return gravity regressor Y_gravity
             */
            static Eigen::Matrix<double, 6, 7> Gravity_Y(Vector6d q);

            void setQPark(const JointState &qpark);
            void setQInit(const JointState &qinit);
            void setQHome(const JointState &qhome);
            bool isRunning() override;

        private:
            bool init();

            bool start();

            bool stop();

            //since this is torque control, update returns joint torques
            Vector6d update(const RobotTime &time, const JointState &current);

            void gravityParameterUpdateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);


            };

    }
}

#endif //HELLOWORLD_GRAVITYCONTROL_H
