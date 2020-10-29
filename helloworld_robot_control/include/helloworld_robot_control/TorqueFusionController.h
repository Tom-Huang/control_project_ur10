//
// Created by helloworld on 28.02.19.
//

#ifndef PROJECT_TORQUEFUSIONCONTROLLER_H
#define PROJECT_TORQUEFUSIONCONTROLLER_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <helloworld_robot_control/ControlDataPublisher.h>
#include <helloworld_robot_control/hasRunning.h>
#include <vector>
#include <helloworld_robot_control/TorqueFusionControlCommander.h>
#include <helloworld_robot_control/TorqueFusionSubcontrollerList.h>


namespace tum_ics_ur_robot_lli{
    namespace RobotControllers {


        /**
        * A controller to fuse multiple other torque controllers together by simple addition (torque=sum(torque_i))
        * @param name controller name
        */
        class TorqueFusionController: public ControlEffort, public ControlDataPublisher, public hasRunning {

        //member variables
        private:

            bool m_startFlag;
            JointState m_qPark;
            JointState m_qHome;
            JointState m_qInit;

            std::vector<tum_ics_ur_robot_lli::RobotControllers::ControlEffort*> controllers;
            ros::ServiceServer commanderService;
            ros::ServiceServer listService;




            //member functions
        public:

            TorqueFusionController(const QString& name="torqueFusionControl");
            ~TorqueFusionController();

            /**
             * Adds and initializes controller. Does not add controllers that cannot be inizialized.
             * @param controller subcontroller who's output should be torque-fused
             * @return false if controller could not be initialized
             */
            bool add(tum_ics_ur_robot_lli::RobotControllers::ControlEffort* controller); //returns false if subcontroller could not be initialized

            void setQInit(const JointState& qinit) override;
            void setQHome(const JointState& qhome) override;
            void setQPark(const JointState& qpark) override;
            bool isRunning() override;

        private:
            bool init() override;
            bool start() override;
            bool stop() override;
            Vector6d update(const RobotTime& time, const JointState &current) override;
            bool commandControllersCallback(helloworld_robot_control::TorqueFusionControlCommander::Request &req, helloworld_robot_control::TorqueFusionControlCommander::Response &resp);
            bool listSubcontrollerCallback(helloworld_robot_control::TorqueFusionSubcontrollerList::Request &req, helloworld_robot_control::TorqueFusionSubcontrollerList::Response &resp);

        };


    }
}



#endif //PROJECT_TORQUEFUSIONCONTROLLER_H
