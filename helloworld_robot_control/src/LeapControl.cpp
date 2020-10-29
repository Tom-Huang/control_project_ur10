#include <helloworld_robot_control/LeapControl.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <math.h>
#include <iostream>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>



namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {
        LeapControl::LeapControl(double weight, const QString &name):
        ControlEffort(name, SPLINE_TYPE, CARTESIAN_SPACE, weight),
        m_startFlag(false),
        running(false),
        m_Kp(Matrix3d::Zero()),
        m_Kd(Matrix6d::Zero()),
        m_Ki(Matrix3d::Zero()),
        N_Kp(Matrix6d::Zero()),
        N_Kd(Matrix6d::Zero()),
        TW_0(Matrix4d::Zero()),
        T0_W(Matrix4d::Zero()),
        Eye(Matrix6d::Zero()),

        m_goal(Vector6d::Zero()),
        m_xStart(Vector6d::Zero()),
        error(Vector3d::Zero()),
        length_vec(Vector6d::Zero()),
        nextGoal(Vector6d::Zero()),
        endstop1(Vector4d::Zero()),
        endstop2(Vector4d::Zero()),

        nextDuration(0.0),
        currentPathBeginTime(0.0),
        m_totalTime(1.0),
        oldTime(0.0)
        {
            pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("helloworld/LeapControl/ControlData",100);
            goalSetterService = n.advertiseService("helloworld/LeapControl/setTrajectory", &LeapControl::setNewTrajectoryCallback, this);
            leapSubscriber = n.subscribe("/helloworld/lm2ros_msg", 1, &LeapControl::leapSubscriberCallback, this);
        }

        LeapControl::~LeapControl() {}

        void LeapControl::setQInit(const JointState& qinit)
        {
            m_qInit=qinit;
        }
        void LeapControl::setQHome(const JointState& qhome)
        {
            m_qHome=qhome;
        }
        void LeapControl::setQPark(const JointState& qpark)
        {
            m_qPark=qpark;
        }


        bool LeapControl::init() {

            std::string currentParamPath = "~LeapControl";
            std::string currentParam;

            if(!ros::param::has(currentParamPath)){
                m_error = true;
                std::string errorString = "Cannot find ros parameter path" + currentParamPath;
                m_errorString = errorString.c_str();
                return false;
            }

            VDouble params;

            //movement velocity and convergence
            currentParam="/commandedVelocity";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=1){
                std::string errorString = this->name().toStdString() + "bad number of velocity parameters: " + std::to_string(params.size()) + "using default from header";
            } else {
                commandedVelocity = params[0];
            }

            currentParam="/reactionTime";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=1){
                std::string errorString = this->name().toStdString() + "bad number of reaction time parameters: " + std::to_string(params.size()) + "using default from header";
            } else {
                reactionTime = params[0];
            }

            //P gains for position control
            currentParam = "/gains_px";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=3){
                m_error = true;
                std::string errorString = this->name().toStdString() + "not enough P gains for position control: " + std::to_string(params.size());
                m_errorString = errorString.c_str();
                return false;
            }
            for(int i = 0; i < 3; i++) m_Kp(i,i) = params[i];
            ROS_INFO_STREAM("Kp_x: " << m_Kp);


            //D gains for position control
            currentParam = "/gains_dx";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=STD_DOF) {
                m_error = true;
                std::string errorString =
                        this->name().toStdString() + "not enough D gains for position control: " + std::to_string(params.size());
                m_errorString = errorString.c_str();
                return false;
            }
            for(int i = 0; i < STD_DOF; i++) m_Kd(i,i) = params[i];
            ROS_INFO_STREAM("Kd_x: " << m_Kd);

            //P gains for nullspace control
            currentParam = "/gains_pn";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=STD_DOF){
                m_error = true;
                std::string errorString = this->name().toStdString() + "not enough P gains for nullspace control: " + std::to_string(params.size());
                m_errorString = errorString.c_str();
                return false;
            }
            for(int i = 0; i < STD_DOF; i++) N_Kp(i,i) = params[i];
            N_Kp = 0.5*N_Kp;
            ROS_INFO_STREAM("Kp_n: " << N_Kp);


            //D gains for nullspace control
            currentParam = "/gains_dn";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=STD_DOF) {
                m_error = true;
                std::string errorString =
                        this->name().toStdString() + "not enough D gains for nullspace control: " + std::to_string(params.size());
                m_errorString = errorString.c_str();
                return false;
            }
            for(int i = 0; i < STD_DOF; i++) N_Kd(i,i) = params[i];
            N_Kd = 0.5*0.75*N_Kd;

            ROS_INFO_STREAM("Kd_n: " << N_Kd);

            // Calculate Ki to be stable
            for(unsigned int i=0; i<3; i++){
                    m_Ki(i,i) = std::pow(m_Kp(i,i),2)/(4*m_Kd(i,i));
            }

            // Define vectors of constants
            Vector6d vec_Eye;

            vec_Eye << 1, 1, 1, 1, 1, 1;
            Eye = vec_Eye.asDiagonal();
            endstop1 << 1, 1, 1, 0;
            endstop1 *= M_PI;
            endstop2 << -1, -1, -1, -1;
            endstop2 *= M_PI;



            //T0_W
            currentParam = "/t0_w";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=16) {
                m_error = true;
                std::string errorString =
                        this->name().toStdString() + "not enough values for world transformation: " + std::to_string(params.size());
                m_errorString = errorString.c_str();
                return false;
            }
            for(int i = 0; i < 4; i++)
                for(unsigned int j = 0; j < 4; j++){
                    T0_W(i,j) = params[i*4+j];
                }
            ROS_INFO_STREAM("T0_w: " << T0_W);

            TW_0 = T0_W.transpose();

            //vector of link lengths
            currentParam = "/length_vec";
            ros::param::get(currentParamPath+currentParam,params);
            if(params.size()!=6) {
                m_error = true;
                std::string errorString =
                        this->name().toStdString() + "not enough link lengths: " + std::to_string(params.size());
                m_errorString = errorString.c_str();
                return false;
            }
            for(int i = 0; i < STD_DOF; i++) length_vec(i) = params[i];
            ROS_INFO_STREAM("length_vec: " << length_vec);

            return true;
        }

        bool LeapControl::start(){
            running = true;
            return true;
        }

        bool LeapControl::stop(){
            running = false;
            m_startFlag = false;

            return true;
        }

        Vector6d LeapControl::update(const RobotTime& time, const JointState &current){

            Vector6d Tau;
            Tau.setZero();
            if(!running) return Tau;//when we're not running, don't output any torque


            // Calculate Current Postion of EF
            Matrix4d TEF_0;
            Matrix4d TEF_W;
            Vector3d X_vec;
            TEF_0 = Transform_EF_W(current.q, length_vec);
            TEF_W = T0_W*TEF_0;


            X_vec = TEF_W.block<3,1>(0,3);

            // Calculate Jacobian, Inverse Jacobian and Manipularibility
            Eigen::Matrix<double,3,6> J;
            Eigen::Matrix<double,6,3> Jinv;
            J = Jacobian(current.q, length_vec).block<3,6>(0,0);
            Jinv = J.transpose()*(J*J.transpose()).inverse();

            Vector3d Xp_vec;
            Vector3d Xp_vec_W;
            Xp_vec_W = J*current.qp;
            Xp_vec = (TW_0*(MatrixXd(4,1) << Xp_vec_W,1).finished()).block<3,1>(0,0);

            if(!m_startFlag){
                m_xStart << Xp_vec, 0,0,0;
                Xd = X_vec;
                m_goal << m_xStart;
                currentPathBeginTime = time.tD();
                m_startFlag = true;
                ROS_INFO_STREAM(TEF_W);
                ROS_INFO_STREAM(TEF_0);
            }

            if((time.tD() - currentPathBeginTime) > m_totalTime && nextDuration > 0.0){
                //initialize waiting trajectory as current
                m_xStart << Xp_vec, 0, 0, 0;
                m_goal = nextGoal;
                m_totalTime = nextDuration;

                currentPathBeginTime = time.tD();
                nextGoal = Vector6d::Zero();
                nextDuration = 0.0;

                error << 0, 0, 0;
                oldTime = 0;

                std::cout << "--------------" << std::endl;
                ROS_INFO_STREAM(current.qp);
                ROS_INFO_STREAM(m_xStart);
                ROS_INFO_STREAM(m_goal);
            }


            // Calculate Desired Position and Velocity of EF
            VVector6d Xd_vec6 = getJointPVT5(m_xStart, m_goal, time.tD() - currentPathBeginTime, m_totalTime);
            VVector3d Xd_vec;
            Xd_vec.append(Xd_vec6[0].block<3,1>(0,0));
            Xd_vec.append(Xd_vec6[1].block<3,1>(0,0));
            Xd_vec.append(Xd_vec6[2].block<3,1>(0,0));

            if(oldTime == 0) {
                Xd = X_vec;
            }
            else{
                Xd += Xd_vec[0]*(time.tD()- oldTime);
//                std::cout << Xp_vec << std::endl;
            }


            // Calulate Position Error of EF
            Vector3d DeltaX;
            DeltaX = Xd - X_vec;

            // Sum error over time
            if(oldTime == 0) {
                error << 0, 0, 0;
            }
            else{
                error = error + DeltaX*(time.tD()- oldTime);
            }

            // Save Current Time
            oldTime = time.tD();

            // Compute Cartesian Reference Velocity
            Vector3d Xpr_0;
            Vector3d Xpr_W;

            Xpr_W = Xd_vec[0] + m_Kp*DeltaX + m_Ki*error;
            Xpr_0 = (TW_0*(MatrixXd(4,1) << Xpr_W,1).finished()).block<3,1>(0,0);




            JacobiSVD<MatrixXd> svd(J);
            double w;
            w = svd.singularValues().prod();

            // PID-like Control
            Vector6d Qpr;
            Vector6d S;
            Qpr = Jinv*Xpr_0;
            S = Jinv*(J*current.qp - Xpr_0);
            Tau = -m_Kd*S;

            // Nullspace Control
            Matrix6d N;
            N = Eye - Jinv*J;

            float weight;
            if(w<0.3){

                weight = (0.3-w)/0.15;
            }
            else{
                weight = 0;
            }

            Vector6d tmp;
            tmp << 0, 0, 0, 0, 0, 0;
            for(unsigned int i = 0; i < 4; i++){
                if(current.q(i) > 0){
                    tmp(i) = 1 - (endstop1(i) - current.q(i));
                }
                else if(current.q(i) < 0){
                    tmp(i) = - 1 + (current.q(i) - endstop2(i));
                }
                else{
                    tmp(i) = 0;
                }
            }
            if(std::abs(current.q(2)) < M_PI/8){
                tmp(2) = (current.q(2)/std::abs(current.q(2)))*(M_PI/8 - std::abs(current.q(2)));
            }

            Tau = Tau + N*(weight*N_Kp*(-tmp) - N_Kd*current.qp);

            //Tau = Tau + N*(weight*current.q.cwiseSign().cwiseProduct(N_Kp*(Pi - 2*(current.q.cwiseAbs()))) - N_Kd*current.qp);


            //publish(time.tD(),(MatrixXd(6,1) << X_vec,Vector3d::Zero()).finished(),(MatrixXd(6,1) << (J * current.qp),Vector3d::Zero()).finished(), Vector6d::Zero(), (MatrixXd(6,1) << Xd_vec[0],Vector3d::Zero()).finished(), (MatrixXd(6,1) << Xd_vec[1],Vector3d::Zero()).finished(), (MatrixXd(6,1) << DeltaX,Vector3d::Zero()).finished(), S, Tau);
            publish(time.tD(),current.q,(current.qp), Vector6d::Zero(), (MatrixXd(6,1) << Xd_vec[0],Vector3d::Zero()).finished(), (MatrixXd(6,1) << Xd_vec[1],Vector3d::Zero()).finished(), (MatrixXd(6,1) << DeltaX,Vector3d::Zero()).finished(), S, Tau);

            //std::cout << w<< std::endl;

            return Tau;

        }


        Eigen::Matrix<double,6,6> LeapControl::Jacobian(Vector6d q, Vector6d l){
            Eigen::Matrix<double,6,6> jacob;
            jacob << l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) - sin(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) + l(3)*cos(q(0)) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)),                       -cos(q(0))*(l(2)*sin(q(1) + q(2)) + l(1)*sin(q(1)) - l(4)*cos(q(1) + q(2) + q(3)) - l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))),                        cos(q(0))*(l(4)*cos(q(1) + q(2) + q(3)) - l(2)*sin(q(1) + q(2)) + l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))), cos(q(0))*(l(4)*cos(q(1) + q(2) + q(3)) + l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))), -l(5)*(sin(q(0))*sin(q(4)) + cos(q(1) + q(2) + q(3))*cos(q(0))*cos(q(4))),                                                     0,
                     l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + cos(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) + l(3)*sin(q(0)) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)),                       -sin(q(0))*(l(2)*sin(q(1) + q(2)) + l(1)*sin(q(1)) - l(4)*cos(q(1) + q(2) + q(3)) - l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))),                        sin(q(0))*(l(4)*cos(q(1) + q(2) + q(3)) - l(2)*sin(q(1) + q(2)) + l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))), sin(q(0))*(l(4)*cos(q(1) + q(2) + q(3)) + l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))),  l(5)*(cos(q(0))*sin(q(4)) - cos(q(1) + q(2) + q(3))*cos(q(4))*sin(q(0))),                                                     0,
                                                                                                                                                                 0, l(2)*cos(q(1) + q(2)) - (l(5)*sin(q(1) + q(2) + q(3) + q(4)))/2 + l(1)*cos(q(1)) + (l(5)*sin(q(1) + q(2) + q(3) - q(4)))/2 + l(4)*sin(q(1) + q(2) + q(3)), l(2)*cos(q(1) + q(2)) - (l(5)*sin(q(1) + q(2) + q(3) + q(4)))/2 + (l(5)*sin(q(1) + q(2) + q(3) - q(4)))/2 + l(4)*sin(q(1) + q(2) + q(3)),           l(4)*sin(q(1) + q(2) + q(3)) - l(5)*cos(q(1) + q(2) + q(3))*sin(q(4)),                             -l(5)*sin(q(1) + q(2) + q(3))*cos(q(4)),                                                     0,
                                                                                                                                                                 0,                                                                                                             sin(q(0)),                                                                                                sin(q(0)),                                                       sin(q(0)),                                 sin(q(1) + q(2) + q(3))*cos(q(0)),   cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)),
                                                                                                                                                                 0,                                                                                                            -cos(q(0)),                                                                                               -cos(q(0)),                                                      -cos(q(0)),                                 sin(q(1) + q(2) + q(3))*sin(q(0)), - cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)),
                                                                                                                                                                 1,                                                                                                                   0,                                                                                                      0,                                                             0,                                        -cos(q(1) + q(2) + q(3)),                            -sin(q(1) + q(2) + q(3))*sin(q(4));


            return jacob;
        }

        Eigen::Matrix<double,4,4> LeapControl::Transform_EF_W(Vector6d q, Vector6d l){
            Eigen::Matrix<double,4,4> transform_ef_w;
            transform_ef_w << cos(q(5))*(sin(q(0))*sin(q(4)) + cos(q(1) + q(2) + q(3))*cos(q(0))*cos(q(4))) - sin(q(1) + q(2) + q(3))*cos(q(0))*sin(q(5)), - sin(q(5))*(sin(q(0))*sin(q(4)) + cos(q(1) + q(2) + q(3))*cos(q(0))*cos(q(4))) - sin(q(1) + q(2) + q(3))*cos(q(0))*cos(q(5)),   cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)), l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + cos(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) + l(3)*sin(q(0)) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)),
                            - cos(q(5))*(cos(q(0))*sin(q(4)) - cos(q(1) + q(2) + q(3))*cos(q(4))*sin(q(0))) - sin(q(1) + q(2) + q(3))*sin(q(0))*sin(q(5)),   sin(q(5))*(cos(q(0))*sin(q(4)) - cos(q(1) + q(2) + q(3))*cos(q(4))*sin(q(0))) - sin(q(1) + q(2) + q(3))*cos(q(5))*sin(q(0)), - cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)), sin(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) - l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) - l(3)*cos(q(0)) + l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)),
                                                                  cos(q(1) + q(2) + q(3))*sin(q(5)) + sin(q(1) + q(2) + q(3))*cos(q(4))*cos(q(5)),                                       cos(q(1) + q(2) + q(3))*cos(q(5)) - sin(q(1) + q(2) + q(3))*cos(q(4))*sin(q(5)),                            -sin(q(1) + q(2) + q(3))*sin(q(4)),                                                       l(0) + l(2)*sin(q(1) + q(2)) + l(1)*sin(q(1)) - l(4)*cos(q(1) + q(2) + q(3)) - l(5)*sin(q(1) + q(2) + q(3))*sin(q(4)),
                                                                                                                              0,                                                                                                   0,                                                     0,                                                                                                                                             1;

            return transform_ef_w;
        }

        bool LeapControl::setNewTrajectoryCallback(helloworld_robot_control::setTrajectoryGoal::Request &req,
                                                  helloworld_robot_control::setTrajectoryGoal::Response &resp) {

            if(req.goal.size() != 6){
                std::stringstream goalPosString;
                goalPosString << "[";
                for(unsigned int i=0; i < req.goal.size(); ++i)goalPosString << req.goal.at(i) << " ";
                goalPosString << "]";
                ROS_ERROR_STREAM("LeapControl: cannot set goal to " << goalPosString.str() << " ! Please provide exactly 6 joint angles!");
                resp.success = false;
                return false;
            }
            Vector6d goal;
            for(int i = 0; i < 6; ++i) goal[i] = req.goal[i];

            bool retVal = setNewTrajectory(goal, req.duration, req.gravityIdentification);
            resp.success = retVal;
            return retVal;
        }

        bool LeapControl::setNewTrajectory(Vector6d goal, double trajectoryDuration, bool identification) {
            nextGoal = goal;
            nextDuration = trajectoryDuration;
            return true;
        }

        bool LeapControl::isRunning() {
            return running;
        }

        void LeapControl::leapSubscriberCallback(const leap_motion::Connect::ConstPtr& msg){
            nextGoal << Vector6d::Zero();
            nextDuration = 0.0;
//            std::cout << msg->is_translation << msg->is_forward << msg->is_right << std::endl;
            if(msg->is_translation){
                if(msg->is_forward < 0) {
                    nextGoal(0) = -commandedVelocity;
                } else if(msg->is_forward > 0){
                    nextGoal(0) = commandedVelocity;
                }

                if(msg->is_right < 0){
                    nextGoal(1) = commandedVelocity;
                } else if(msg->is_right > 0){
                    nextGoal(1) = -commandedVelocity;
                }

                if(msg->is_up < 0){
                    nextGoal(2) = -commandedVelocity;
                } else if(msg->is_up > 0){
                    nextGoal(2) = commandedVelocity;
                }
                nextDuration = reactionTime;
            }

        }



    }
}