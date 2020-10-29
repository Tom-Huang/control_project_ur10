#include <helloworld_robot_control/GravityControl.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <std_msgs/MultiArrayDimension.h>
#include <math.h>


namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

    GravityControl::GravityControl(const QString &name) : ControlEffort(name,SPLINE_TYPE, JOINT_SPACE),
        m_startFlag(false),
        running(false),
        identificationFlag(false) {
        pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("helloworld/GravityControl/ControlData",100);
        subGravityParameterUpdates = n.subscribe("helloworld/gravityParameter_updates", 1, &GravityControl::gravityParameterUpdateCallback, this);
    }

    GravityControl::~GravityControl() {}

    bool GravityControl::init(){

        std::string currentParamPath = "~GravityControl";
        std::string currentParam;

        VDouble params;

        //gravity Parameters
        currentParam = "/gravityParameters";
        ros::param::get(currentParamPath+currentParam,params);
        if(params.size()!=7){
            m_error = true;
            std::string errorString = this->name().toStdString() + "not enough gravity parameters: " + std::to_string(params.size());
            m_errorString = errorString.c_str();
            return false;
        }
        for(int i = 0; i < 7; i++) gravityParameters[i] = params[i];
        ROS_INFO_STREAM("Kd: " << gravityParameters);

        return start();
    }

    bool GravityControl::start(){
        running = true;
        return true;
    }

    bool GravityControl::stop(){
        std::cout << "GravityControl stopped!" << std::endl;
        running = false;
        m_startFlag = false;
        return true;
    }

    Vector6d GravityControl::update(const RobotTime& time, const JointState &current){
        Vector6d tau;
        tau.setZero();

        if(!running) return tau;

        if(!m_startFlag) m_startFlag = true;


        tau = GravityControl::Gravity_Y(current.q) * gravityParameters;

        publish(time.tD(), current.q, current.qp, current.qpp, current.q, current.qp, Vector6d::Zero(), Vector6d::Zero(), tau);
        return tau;
    }

    void GravityControl::gravityParameterUpdateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
        if(msg->layout.dim[0].size != 7) {
            ROS_ERROR_STREAM("Recieved message with bad parameter count: " << msg);
            return;
        }
        for(int i = 0; i<7; ++i) gravityParameters[i] = msg->data.at(i);//TODO test this

    }

    Eigen::Matrix<double,6,7> GravityControl::Gravity_Y(Vector6d q){
        Eigen::Matrix<double,6,7> g_y;
        g_y << 0, 0, 0, 0, 0, 0, 0, std::cos(q(1)), std::cos(q(1))*std::cos(q(2)) - std::sin(q(1))*std::sin(q(2)), std::sin(q(1))*std::sin(q(2))*std::sin(q(3)) - std::cos(q(1))*std::cos(q(2))*std::sin(q(3)) - std::cos(q(1))*std::cos(q(3))*std::sin(q(2)) - std::cos(q(2))*std::cos(q(3))*std::sin(q(1)), -std::cos(q(1)), std::sin(q(1))*std::sin(q(2)) - std::cos(q(1))*std::cos(q(2)), std::cos(q(1))*std::cos(q(2))*std::sin(q(3)) - std::sin(q(1))*std::sin(q(2))*std::sin(q(3)) + std::cos(q(1))*std::cos(q(3))*std::sin(q(2)) + std::cos(q(2))*std::cos(q(3))*std::sin(q(1)), std::cos(q(1))*std::cos(q(2))*std::cos(q(3))*std::sin(q(4)) - std::cos(q(1))*std::sin(q(2))*std::sin(q(3))*std::sin(q(4)) - std::cos(q(2))*std::sin(q(1))*std::sin(q(3))*std::sin(q(4)) - std::cos(q(3))*std::sin(q(1))*std::sin(q(2))*std::sin(q(4)),        0, std::cos(q(1))*std::cos(q(2)) - std::sin(q(1))*std::sin(q(2)), std::sin(q(1))*std::sin(q(2))*std::sin(q(3)) - std::cos(q(1))*std::cos(q(2))*std::sin(q(3)) - std::cos(q(1))*std::cos(q(3))*std::sin(q(2)) - std::cos(q(2))*std::cos(q(3))*std::sin(q(1)),        0, std::sin(q(1))*std::sin(q(2)) - std::cos(q(1))*std::cos(q(2)), std::cos(q(1))*std::cos(q(2))*std::sin(q(3)) - std::sin(q(1))*std::sin(q(2))*std::sin(q(3)) + std::cos(q(1))*std::cos(q(3))*std::sin(q(2)) + std::cos(q(2))*std::cos(q(3))*std::sin(q(1)), std::cos(q(1))*std::cos(q(2))*std::cos(q(3))*std::sin(q(4)) - std::cos(q(1))*std::sin(q(2))*std::sin(q(3))*std::sin(q(4)) - std::cos(q(2))*std::sin(q(1))*std::sin(q(3))*std::sin(q(4)) - std::cos(q(3))*std::sin(q(1))*std::sin(q(2))*std::sin(q(4)),        0,                                 0, std::sin(q(1))*std::sin(q(2))*std::sin(q(3)) - std::cos(q(1))*std::cos(q(2))*std::sin(q(3)) - std::cos(q(1))*std::cos(q(3))*std::sin(q(2)) - std::cos(q(2))*std::cos(q(3))*std::sin(q(1)),        0,                                 0, std::cos(q(1))*std::cos(q(2))*std::sin(q(3)) - std::sin(q(1))*std::sin(q(2))*std::sin(q(3)) + std::cos(q(1))*std::cos(q(3))*std::sin(q(2)) + std::cos(q(2))*std::cos(q(3))*std::sin(q(1)), std::cos(q(1))*std::cos(q(2))*std::cos(q(3))*std::sin(q(4)) - std::cos(q(1))*std::sin(q(2))*std::sin(q(3))*std::sin(q(4)) - std::cos(q(2))*std::sin(q(1))*std::sin(q(3))*std::sin(q(4)) - std::cos(q(3))*std::sin(q(1))*std::sin(q(2))*std::sin(q(4)),        0,                                 0,                                                                                                     0,        0,                                 0,                                                                                                     0, std::cos(q(1))*std::cos(q(2))*std::cos(q(4))*std::sin(q(3)) + std::cos(q(1))*std::cos(q(3))*std::cos(q(4))*std::sin(q(2)) + std::cos(q(2))*std::cos(q(3))*std::cos(q(4))*std::sin(q(1)) - std::cos(q(4))*std::sin(q(1))*std::sin(q(2))*std::sin(q(3)),        0,                                 0,                                                                                                     0,        0,                                 0,                                                                                                     0,                                                                                                                                     0;
        return g_y;
    }

        void GravityControl::setQInit(const JointState& qinit)
        {
            m_qInit=qinit;
        }
        void GravityControl::setQHome(const JointState& qhome)
        {
            m_qHome=qhome;
        }
        void GravityControl::setQPark(const JointState& qpark)
        {
            m_qPark=qpark;
        }

        bool GravityControl::isRunning(){
            return running;
        }

    }
}