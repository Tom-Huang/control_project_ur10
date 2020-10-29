#include <helloworld_robot_control/TorqueFusionController.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <string>
#include <iostream>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        TorqueFusionController::TorqueFusionController(const QString &name): ControlEffort (name, SPLINE_TYPE, JOINT_SPACE),
        m_startFlag(false) {
            pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("helloworld/torqueFusionControl/ControlData",100);
            commanderService = n.advertiseService("helloworld/torqueFusionControl/commandSubcontrollers", &TorqueFusionController::commandControllersCallback, this);
            listService = n.advertiseService("helloworld/torqueFusionControl/listSubcontrollers", &TorqueFusionController::listSubcontrollerCallback, this);
        }

        TorqueFusionController::~TorqueFusionController() {}

        bool TorqueFusionController::init(){
            if(m_qHome.q.isZero()){
                m_error = true;
                m_errorString = "qHome not initialized";
                return false;
            }
            return start();
        }

        bool TorqueFusionController::start(){
            if(m_startFlag) return true;//base control already running, please start controllers you added later manually
            //check if any subcontrollers are present
            if(controllers.empty()){
                m_error = true;
                std::string errorString = "No subcontrollerswere added to " + this->name().toStdString() + "!";
                m_errorString = errorString.c_str();
                return false;
            }

//            //start subcontrollers and abort if any of them could not be started
//            for(tum_ics_ur_robot_lli::RobotControllers::ControlEffort* ctrl : controllers) {
//                if(!ctrl->start()){
//                    for(tum_ics_ur_robot_lli::RobotControllers::ControlEffort* ctrlToBeStopped : controllers) ctrlToBeStopped->stop();
//                    m_error = true;
//                    std::string errorString = "Could not start subcontroller " + ctrl->name().toStdString() + " !";
//                    m_errorString = errorString.c_str();
//                    return false;
//                }
//            }

            m_startFlag = true;
            return true;
            }

        bool TorqueFusionController::stop() {
            return false; //base controller should never be stopped
        }

        Vector6d TorqueFusionController::update(const RobotTime& time, const JointState &current) {

            Vector6d tau;
            tau.setZero();

            if(!m_startFlag){//check if controller is running
                m_error = true;
                std::string errorString = "Cannot update: TorqueFusionController " + this->name().toStdString() + " has not been started yet!";
                m_errorString = errorString.c_str();
                return tau; //should be zero
            }


            //additive control fusion: tau_total = sum(tau_i), all i
            for(tum_ics_ur_robot_lli::RobotControllers::ControlEffort* ctrl : controllers) {
                if(ctrl->error()){
                    ROS_ERROR_STREAM(ctrl->name().toStdString() << "%s error: %s"<< ctrl->errorString().toStdString());
                }
                tau += ctrl->update(time, current);
            }
            publish(time.tD(), current.q, current.qp, current.qpp, Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero(), tau);
            return tau;
        }



        bool TorqueFusionController::add(tum_ics_ur_robot_lli::RobotControllers::ControlEffort* ctrl) {
            ctrl->setQHome(m_qHome);
            ctrl->setQPark(m_qPark);
            if(!ctrl->init()){
                ROS_ERROR_STREAM(this->name().toStdString() << ": cannot add " << ctrl->name().toStdString() << "! Error: " << ctrl->errorString().toStdString());
                return false;
            }
            controllers.push_back(ctrl);
            return true;
        }

        void TorqueFusionController::setQInit(const JointState& qinit)
        {
            m_qInit=qinit;
        }
        void TorqueFusionController::setQHome(const JointState& qhome)
        {
            m_qHome=qhome;
        }
        void TorqueFusionController::setQPark(const JointState& qpark)
        {
            m_qPark=qpark;
        }

        bool TorqueFusionController::commandControllersCallback(helloworld_robot_control::TorqueFusionControlCommander::Request &req, helloworld_robot_control::TorqueFusionControlCommander::Response &resp){
            std::string stopMe = req.ctrlStop;
            std::string startMe = req.ctrlStart;
            if(!stopMe.compare(startMe)){
                resp.stopSuccess = false;
                resp.startSuccess = false;
                return false;
            }

            tum_ics_ur_robot_lli::RobotControllers::ControlEffort* stopCtrl = nullptr;
            tum_ics_ur_robot_lli::RobotControllers::ControlEffort* startCtrl = nullptr;

            for(tum_ics_ur_robot_lli::RobotControllers::ControlEffort* ctrl: controllers){
                if(!ctrl->name().toStdString().compare(stopMe)) stopCtrl = ctrl;
                if(!ctrl->name().toStdString().compare(startMe)) startCtrl = ctrl;
            }

            resp.stopSuccess = (stopCtrl == nullptr) ? true : stopCtrl->stop();
            resp.startSuccess = (startCtrl == nullptr) ? true : startCtrl->start();

            return resp.stopSuccess && resp.startSuccess;
        }

        bool TorqueFusionController::listSubcontrollerCallback(helloworld_robot_control::TorqueFusionSubcontrollerList::Request &req, helloworld_robot_control::TorqueFusionSubcontrollerList::Response &resp){
            //dirty hack
            for(tum_ics_ur_robot_lli::RobotControllers::ControlEffort* ct : controllers) {
                std::string state = "-";
                try {
                    tum_ics_ur_robot_lli::RobotControllers::hasRunning* ctrl = dynamic_cast<tum_ics_ur_robot_lli::RobotControllers::hasRunning*>(ct);
                    if(ctrl != nullptr) state = (ctrl->isRunning())?":ON":":OFF";
                } catch (const std::exception& e) {
                    state = "-";
                }
                std::string controllerAndState = ct->name().toStdString() + state;
                resp.list.push_back(controllerAndState);
            }
            return true;
        }

        bool TorqueFusionController::isRunning() {
            return m_startFlag;
        }


    }
}
