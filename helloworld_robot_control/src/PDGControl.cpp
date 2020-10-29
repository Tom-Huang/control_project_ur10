#include <helloworld_robot_control/PDGControl.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <math.h>
#include <iostream>
#include <helloworld_robot_control/GravityControl.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

//TODO goal,totalTime&ident setterService

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {
        PDGControl::PDGControl(const QString &name): ControlEffort(name, SPLINE_TYPE, JOINT_SPACE),
        m_startFlag(false),
        running(false),
        identificationFlag(false),
        nextIdentification(false),
        moving(false),
        m_Kp(Matrix6d::Zero()),
        m_Kd(Matrix6d::Zero()),
        m_qStart(Vector6d::Zero()),
        m_goal(Vector6d::Zero()),
        nextGoal(Vector6d::Zero()),
        m_totalTime(1.0),
        nextDuration(0.0),
        currentPathBeginTime(0.0),
        gravityParameters(Vector7d::Zero()){
            pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("helloworld/PDControl/ControlData",100);
            pubGravityParameterUpdates = n.advertise<std_msgs::Float64MultiArray>("helloworld/gravityParameter_updates", 1);
            goalSetterService = n.advertiseService("helloworld/PDControl/setTrajectory", &PDGControl::setNewTrajectoryCallback, this);
        }

        PDGControl::~PDGControl(){

        }




        bool PDGControl::init() {
            if(m_qHome.q.isZero()){
                m_error = true;
                m_errorString = "qHome not initialized";
                return false;
            }
            m_goal= m_qHome.q;//default goal: home;

            std::string currentParamPath = "~PDControl";
            std::string currentParam;

            if(!ros::param::has(currentParamPath)){
                m_error = true;
                std::string errorString = "Cannot find ros parameter path" + currentParamPath;
                m_errorString = errorString.c_str();
                return false;
            }

            VDouble params;


            //P gains
            currentParam = "/gains_p";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=STD_DOF){
                m_error = true;
                std::string errorString = this->name().toStdString() + "not enough P gains: " + std::to_string(params.size());
                m_errorString = errorString.c_str();
                return false;
            }
            for(int i = 0; i < STD_DOF; i++) m_Kp(i,i) = params[i];
            ROS_INFO_STREAM("Kp: " << m_Kp);


            //D gains
            currentParam = "/gains_d";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=STD_DOF){
                m_error = true;
                std::string errorString = this->name().toStdString() + "not enough D gains: " + std::to_string(params.size());
                m_errorString = errorString.c_str();
                return false;
            }
            for(int i = 0; i < STD_DOF; i++) m_Kd(i,i) = params[i];
            ROS_INFO_STREAM("Kd: " << m_Kd);

            //gravity Parameters
            VDouble params2;
            ros::param::get("~GravityControl/gravityParameters",params);
            if(params.size()!=7){
                m_error = true;
                std::string errorString = this->name().toStdString() + "not enough gravity parameters: " + std::to_string(params.size());
                m_errorString = errorString.c_str();
                return false;
            }
            for(int i = 0; i < 7; i++) gravityParameters[i] = params[i];
            ROS_INFO_STREAM("Kd: " << gravityParameters);


            return start();//TODO dont default start after implementing start via service
        }

        bool PDGControl::start(){
            running = true;
            return true;
        }

        bool PDGControl::stop(){
            running = false;
            m_startFlag = false;
            return true;
        }

        Vector6d PDGControl::update(const RobotTime& time, const JointState &current){
            Vector6d tau;
            tau.setZero();
            if(!running) return tau;//when we're not running, don't output any torque

            if(!m_startFlag){//i'd rather only do this via startflag
                m_qStart = current.q;//begin control at current configuration
                currentPathBeginTime = time.tD();
                m_startFlag = true;
            }

            if((time.tD() - currentPathBeginTime) > m_totalTime && nextDuration > 0.0){
                //initialize waiting trajectory as current
                m_qStart = current.q;
                m_goal = nextGoal;
                m_totalTime = nextDuration;

                currentPathBeginTime = time.tD();
                nextGoal = Vector6d::Zero();
                nextDuration = 0.0;
            }

            VVector6d Qd_vec = getJointPVT5(m_qStart, m_goal, time.tD() - currentPathBeginTime, m_totalTime);

            Vector6d deltaQ = Qd_vec[0] - current.q;
            Vector6d deltaQp = current.qp - Qd_vec[1];

            JointState js_r = current;
            js_r.qp = Qd_vec[1] + m_Kp * deltaQ;
            js_r.qpp = Qd_vec[2] + m_Kp * deltaQp;

            Vector6d S = current.qp - js_r.qp;
            for (int i = 0; i < S.size(); i++) if (std::abs(S(i)) <= 0.000001) S(i) = 0;//to reduce very small oscillations control cannot get rid off


            if(identificationFlag) {
                double stepsize = 0.0105/(1+0.0000025*time.tD());
                gravityParameters = gravityParameters - stepsize * GravityControl::Gravity_Y(current.q).transpose() * S;

                std_msgs::Float64MultiArray msg;
                msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                msg.layout.dim[0].label = "gravityParameters";
                msg.layout.dim[0].size = 7;
                msg.layout.dim[0].stride = 7;


                for (int i = 1; i < 7; ++i) msg.data.push_back(gravityParameters[i]);
                pubGravityParameterUpdates.publish(msg);
            }


            tau = -m_Kd * S;

            publish(time.tD(), js_r.qp, js_r.qp, js_r.qpp, Qd_vec[0], Qd_vec[1], deltaQ, deltaQp, tau);

            return tau;
        }

        void PDGControl::setQInit(const JointState& qinit)
        {
            m_qInit=qinit;
        }
        void PDGControl::setQHome(const JointState& qhome)
        {
            if(m_goal.isApprox(m_qHome.q)) m_goal = qhome.q;//TODO test this
            m_qHome=qhome;
        }
        void PDGControl::setQPark(const JointState& qpark)
        {
            m_qPark=qpark;
        }

        bool PDGControl::setNewTrajectoryCallback(helloworld_robot_control::setTrajectoryGoal::Request &req,
                                                  helloworld_robot_control::setTrajectoryGoal::Response &resp) {

            if(req.goal.size() != 6){
                std::stringstream goalPosString;
                goalPosString << "[";
                for(unsigned int i=0; i < req.goal.size(); ++i)goalPosString << req.goal.at(i) << " ";
                goalPosString << "]";
                ROS_ERROR_STREAM("PDControl: cannot set goal to " << goalPosString.str() << " ! Please provide exactly 6 joint angles!");
                resp.success = false;
                return false;
            }
            Vector6d goal;
            for(int i = 0; i < 6; ++i) goal[i] = DEG2RAD(req.goal[i]);

            bool retVal = setNewTrajectory(goal, req.duration, req.gravityIdentification);
            resp.success = retVal;
            return retVal;
        }

        bool PDGControl::setNewTrajectory(Vector6d goal, double trajectoryDuration, bool identification) {
            nextGoal = goal;
            nextDuration = trajectoryDuration;
            nextIdentification = identification;
            return true;
        }

        bool PDGControl::isRunning() {
            return running;
        }
    }
}