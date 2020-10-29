#include <helloworld_robot_control/OrientationLeapControl.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <math.h>
#include <iostream>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <cv_tracker/destination_msg.h>



namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {
        OrientationLeapControl::OrientationLeapControl(double weight, const QString &name):
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
        m_Start(Vector6d::Zero()),
        error(Vector3d::Zero()),
        error1(Vector3d::Zero()),
        length_vec(Vector6d::Zero()),
        nextGoal(Vector6d::Zero()),
        endstop1(Vector4d::Zero()),
        endstop2(Vector4d::Zero()),
        Xd_vec(Vector3d::Zero()),


        nextDuration(0.0),
        currentPathBeginTime(0.0),
        m_totalTime(1.0),
        oldTime(0.0)
        {
            pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("helloworld/OrientationLeapControl/ControlData",100);
            goalSetterService = n.advertiseService("helloworld/OrientationLeapControl/setTrajectory", &OrientationLeapControl::setNewTrajectoryCallback, this);
            leapSubscriber = n.subscribe("helloworld/lm2ros_msg", 1, &OrientationLeapControl::leapSubscriberCallback, this);

        }

        OrientationLeapControl::~OrientationLeapControl() {}

        void OrientationLeapControl::setQInit(const JointState& qinit)
        {
            m_qInit=qinit;
        }
        void OrientationLeapControl::setQHome(const JointState& qhome)
        {
            m_qHome=qhome;
        }
        void OrientationLeapControl::setQPark(const JointState& qpark)
        {
            m_qPark=qpark;
        }


        bool OrientationLeapControl::init() {

            std::string currentParamPath = "~OrientationLeapControl";
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

        bool OrientationLeapControl::start(){
            running = true;


            return true;
        }

        bool OrientationLeapControl::stop(){
            running = false;
            m_startFlag = false;
            return true;
        }

        Vector6d OrientationLeapControl::update(const RobotTime& time, const JointState &current){

            Vector6d Tau;
            Tau.setZero();
            if(!running) return Tau;//when we're not running, don't output any torque


            // Calculate Current Postion of EF
            Matrix4d TEF_0;
            Matrix4d TEF_W;
            Vector3d X_vec;
            TEF_0 = Transform_EF_0(current.q, length_vec);
            TEF_W = T0_W*TEF_0;

            X_vec = TEF_W.block<3,1>(0,3);

            // Calculate current orientation in quaternions
            Vector4d lambda4;
            Vector3d lambda;


            Matrix3d rot = TEF_W.block<3,3>(0,0);
            Eigen::Quaterniond quat(rot);
            lambda4[0] = quat.w();
            lambda4[1] = quat.vec()[0];
            lambda4[2] = quat.vec()[1];
            lambda4[3] = quat.vec()[2];
//            std::cout << lambda4[0] << std::endl;

            lambda =  lambda4.block<3,1>(1,0);

            // Calculate Jacobian, Inverse Jacobian and Manipularibility
            Eigen::Matrix<double,3,6> J;
            Eigen::Matrix<double,6,3> Jinv;
            J = Jacobian(current.q, length_vec).block<3,6>(0,0);
            Jinv = J.transpose()*(J*J.transpose()).inverse();

            // Angular Velocity Jacobian
            Eigen::Matrix<double,3,6> JW;
            Eigen::Matrix<double,6,3> JWInv;
            JW = Jacobian(current.q, length_vec).block<3,6>(3,0);
            JWInv = JW.transpose()*(JW*JW.transpose()).inverse();

            // Orientation Jacobian
            Matrix3d JLambda;
            Matrix3d JLambdaInv;
            JLambda = JacobianLambda(lambda4);
            JLambdaInv = JLambda.inverse();

            Vector3d Xp_vec;
            Vector3d Lambdap_vec;
            Vector3d Xp_vec_W;
            Xp_vec_W = J*current.qp;
            Xp_vec = (TW_0*(MatrixXd(4,1) << Xp_vec_W,1).finished()).block<3,1>(0,0);


            if(!m_startFlag){
                m_Start << Xp_vec, 0,0,0;
                Xd = X_vec;
                Lambdad = lambda;
                m_goal << m_Start;
                currentPathBeginTime = time.tD();
                m_startFlag = true;

            }

            if((time.tD() - currentPathBeginTime) > m_totalTime && nextDuration > 0.0){
                //initialize waiting trajectory as current
                m_Start << Xp_vec, 0,0,0;
                m_goal = nextGoal;
                m_totalTime = nextDuration;

                currentPathBeginTime = time.tD();
                nextGoal = Vector6d::Zero();
                nextDuration = 0.0;

                error1 << 0, 0, 0;
                error << 0, 0, 0;
                oldTime = 0;
            }


            // Calculate Desired Position and Velocity of EF
            VVector6d trajectory_vec6 = getJointPVT5(m_Start, m_goal, time.tD() - currentPathBeginTime, m_totalTime);
            VVector3d Xd_vec;

            Xd_vec.append(trajectory_vec6[0].block<3,1>(0,0));
            Xd_vec.append(trajectory_vec6[1].block<3,1>(0,0));
            Xd_vec.append(trajectory_vec6[2].block<3,1>(0,0));


            if(oldTime == 0) {
                Xd = X_vec;
            }
            else{
                Xd += Xd_vec[0]*(time.tD()- oldTime);
            }

            // Calulate Position and Orientation Error of EF
            Vector3d DeltaX;
            Vector3d DeltaLambda;
            DeltaX = Xd - X_vec;
            DeltaLambda = Lambdad- lambda;

            // Sum error over time
            if(oldTime == 0) {
                error << 0, 0, 0;
                error1 << 0, 0, 0;
            }
            else{
                error = error + DeltaX*(time.tD()- oldTime);
                error1 = error1 + DeltaLambda*(time.tD()- oldTime);

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
            Vector6d S;
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


            Tau = Tau + N*(0*weight*N_Kp*(tmp) - N_Kd*current.qp);




            Matrix3d KpLambda;
            Matrix6d KdLambda;

            KpLambda = Matrix3d::Zero();
            KdLambda = Matrix6d::Zero();
//            KpLambda(0,0) = 20;
//            KpLambda(1,1) = 20;
//            KpLambda(2,2) = 20;
//
//            KdLambda(0,0) = 15;
//            KdLambda(1,1) = 15;
//            KdLambda(2,2) = 15;
//            KdLambda(3,3) = 15;
//            KdLambda(4,4) = 10;
//            KdLambda(5,5) = 10;
//
//            KdLambda *=0.5;
//            KpLambda *=1.1;

//            for(unsigned int i=0; i<3; i++){
//                KpLambda(i,i) = (KpLambda(i,i))/(KdLambda(i,i));
//            }


            KpLambda(0,0) = 70;
            KpLambda(1,1) = 70;
            KpLambda(2,2) = 51;

            KdLambda(0,0) = 12.5;
            KdLambda(1,1) = 10;
            KdLambda(2,2) = 10;
            KdLambda(3,3) = 10;
            KdLambda(4,4) = 10;
            KdLambda(5,5) = 10;

            KdLambda *= 0.80;
            KpLambda *= 0.5;



            // Calculate Ki to be stable
            Matrix3d KiLambda;
            KiLambda = Matrix3d::Zero();
            for(unsigned int i=0; i<3; i++){
                KiLambda(i,i) = std::pow(KpLambda(i,i),2)/(4*KdLambda(i,i));
            }

            // Velocity Reference
            Vector3d Lambdapr_W;
            Vector3d Lambdapr_0;
            Lambdapr_W = KpLambda*DeltaLambda;//+ KiLambda*error1;
            Lambdapr_0 = (TW_0*(MatrixXd(4,1) << Lambdapr_W,1).finished()).block<3,1>(0,0);

            //std::cout << w<< std::endl;


//            std::cout << DeltaLambda << std::endl;
//            std::cout << "+++++++++++++"<< std::endl;
//            std::cout << S << std::endl;
//            std::cout << "----------"<< std::endl;

//            std::cout << "''''''''''''''\n" << N << std::endl;
//            if(JLambda.determinant() == 0) {
//                std::cout << "JLambda: \n" << JLambda << std::endl;
//            }
//            if(JW.determinant() == 0) {
//                std::cout << "JW: \n" << JW << std::endl;
//            }

            S = JWInv*JLambdaInv*(JLambda*JW*current.qp - Lambdapr_0);

//            ROS_INFO_STREAM(S(4));
            // KdLambda(4,4)= 1;
            //KdLambda(5,5)= 1;

            for(unsigned int i=0; i<6; i++){
                if(std::abs(S(i))<std::pow(10,-6)) S(i) = 0;
            }

            Tau = Tau - KdLambda*S;






//            std::cout << Tau<< std::endl;


//                ROS_INFO_STREAM(lambdad_v[1]);
//                std::cout << "---------------------------" << std::endl;
//                ROS_INFO_STREAM(KiLambda);
//                std::cout << "---------------------------" << std::endl;
//                ROS_INFO_STREAM(error1);
//                std::cout << "---------------------------" << std::endl;
//                ROS_INFO_STREAM(JLambda*JW*current.qp - Lambdapr_0);
//                std::cout << "---------------------------" << std::endl;
//                ROS_INFO_STREAM(error1);
//                std::cout << "---------------------------" << std::endl;
//                ROS_INFO_STREAM(S);
//
//                std::cout << "########################" << std::endl;
            return Tau;

        }


        Eigen::Matrix<double,6,6> OrientationLeapControl::Jacobian(Vector6d q, Vector6d l){
            Eigen::Matrix<double,6,6> jacob;
            jacob << l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) - sin(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) + l(3)*cos(q(0)) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)),                       -cos(q(0))*(l(2)*sin(q(1) + q(2)) + l(1)*sin(q(1)) - l(4)*cos(q(1) + q(2) + q(3)) - l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))),                        cos(q(0))*(l(4)*cos(q(1) + q(2) + q(3)) - l(2)*sin(q(1) + q(2)) + l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))), cos(q(0))*(l(4)*cos(q(1) + q(2) + q(3)) + l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))), -l(5)*(sin(q(0))*sin(q(4)) + cos(q(1) + q(2) + q(3))*cos(q(0))*cos(q(4))),                                                     0,
                     l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + cos(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) + l(3)*sin(q(0)) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)),                       -sin(q(0))*(l(2)*sin(q(1) + q(2)) + l(1)*sin(q(1)) - l(4)*cos(q(1) + q(2) + q(3)) - l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))),                        sin(q(0))*(l(4)*cos(q(1) + q(2) + q(3)) - l(2)*sin(q(1) + q(2)) + l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))), sin(q(0))*(l(4)*cos(q(1) + q(2) + q(3)) + l(5)*sin(q(1) + q(2) + q(3))*sin(q(4))),  l(5)*(cos(q(0))*sin(q(4)) - cos(q(1) + q(2) + q(3))*cos(q(4))*sin(q(0))),                                                     0,
                                                                                                                                                                 0, l(2)*cos(q(1) + q(2)) - (l(5)*sin(q(1) + q(2) + q(3) + q(4)))/2 + l(1)*cos(q(1)) + (l(5)*sin(q(1) + q(2) + q(3) - q(4)))/2 + l(4)*sin(q(1) + q(2) + q(3)), l(2)*cos(q(1) + q(2)) - (l(5)*sin(q(1) + q(2) + q(3) + q(4)))/2 + (l(5)*sin(q(1) + q(2) + q(3) - q(4)))/2 + l(4)*sin(q(1) + q(2) + q(3)),           l(4)*sin(q(1) + q(2) + q(3)) - l(5)*cos(q(1) + q(2) + q(3))*sin(q(4)),                             -l(5)*sin(q(1) + q(2) + q(3))*cos(q(4)),                                                     0,
                                                                                                                                                                 0,                                                                                                             sin(q(0)),                                                                                                sin(q(0)),                                                       sin(q(0)),                                 sin(q(1) + q(2) + q(3))*cos(q(0)),   cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)),
                                                                                                                                                                 0,                                                                                                            -cos(q(0)),                                                                                               -cos(q(0)),                                                      -cos(q(0)),                                 sin(q(1) + q(2) + q(3))*sin(q(0)), - cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)),
                                                                                                                                                                 1,                                                                                                                   0,                                                                                                      0,                                                             0,                                        -cos(q(1) + q(2) + q(3)),                            -sin(q(1) + q(2) + q(3))*sin(q(4));


            return jacob;
        }

        Eigen::Matrix<double,3,3> OrientationLeapControl::JacobianLambda(Vector4d lambda){
            Eigen::Matrix<double,3,3> jacobLambda;
            jacobLambda << lambda(0), lambda(3), -lambda(2),
                          -lambda(3), lambda(0), lambda(1),
                           lambda(2), -lambda(1), lambda(0);

            return jacobLambda;
        }

        Eigen::Matrix<double,4,4> OrientationLeapControl::Transform_EF_0(Vector6d q, Vector6d l){
            Eigen::Matrix<double,4,4> transform_ef_0;
            transform_ef_0 << cos(q(5))*(sin(q(0))*sin(q(4)) + cos(q(1) + q(2) + q(3))*cos(q(0))*cos(q(4))) - sin(q(1) + q(2) + q(3))*cos(q(0))*sin(q(5)), - sin(q(5))*(sin(q(0))*sin(q(4)) + cos(q(1) + q(2) + q(3))*cos(q(0))*cos(q(4))) - sin(q(1) + q(2) + q(3))*cos(q(0))*cos(q(5)),   cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)), l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + cos(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) + l(3)*sin(q(0)) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)),
                            - cos(q(5))*(cos(q(0))*sin(q(4)) - cos(q(1) + q(2) + q(3))*cos(q(4))*sin(q(0))) - sin(q(1) + q(2) + q(3))*sin(q(0))*sin(q(5)),   sin(q(5))*(cos(q(0))*sin(q(4)) - cos(q(1) + q(2) + q(3))*cos(q(4))*sin(q(0))) - sin(q(1) + q(2) + q(3))*cos(q(5))*sin(q(0)), - cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)), sin(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) - l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) - l(3)*cos(q(0)) + l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)),
                                                                  cos(q(1) + q(2) + q(3))*sin(q(5)) + sin(q(1) + q(2) + q(3))*cos(q(4))*cos(q(5)),                                       cos(q(1) + q(2) + q(3))*cos(q(5)) - sin(q(1) + q(2) + q(3))*cos(q(4))*sin(q(5)),                            -sin(q(1) + q(2) + q(3))*sin(q(4)),                                                       l(0) + l(2)*sin(q(1) + q(2)) + l(1)*sin(q(1)) - l(4)*cos(q(1) + q(2) + q(3)) - l(5)*sin(q(1) + q(2) + q(3))*sin(q(4)),
                                                                                                                              0,                                                                                                   0,                                                     0,                                                                                                                                             1;

            return transform_ef_0;
        }

        bool OrientationLeapControl::setNewTrajectoryCallback(helloworld_robot_control::setTrajectoryGoal::Request &req,
                                                  helloworld_robot_control::setTrajectoryGoal::Response &resp) {

            if(req.goal.size() != 6){
                std::stringstream goalPosString;
                goalPosString << "[";
                for(unsigned int i=0; i < req.goal.size(); ++i)goalPosString << req.goal.at(i) << " ";
                goalPosString << "]";
                ROS_ERROR_STREAM("OrientationLeapControl: cannot set goal to " << goalPosString.str() << " ! Please provide exactly 6 joint angles!");
                resp.success = false;
                return false;
            }
            Vector6d goal;
            for(int i = 0; i < 6; ++i) goal[i] = req.goal[i];

            bool retVal = setNewTrajectory(goal, req.duration, req.gravityIdentification);
            resp.success = retVal;
            return retVal;
        }

        bool OrientationLeapControl::setNewTrajectory(Vector6d goal, double trajectoryDuration, bool identification) {
            nextGoal = goal;
            ROS_INFO_STREAM("goalsetter sets " << goal);
            nextDuration = trajectoryDuration;
            return true;
        }

        bool OrientationLeapControl::isRunning() {
            return running;
        }

        void OrientationLeapControl::leapSubscriberCallback(const leap_motion::Connect::ConstPtr& msg){
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

            }
            else{
                if(msg->is_roll < 0) {
                    nextGoal(3) = -commandedVelocity/10;
                } else if(msg->is_roll > 0){
                    nextGoal(3) = commandedVelocity/10;
                }

                if(msg->is_pitch < 0){
                    nextGoal(4) = -commandedVelocity/10;
                } else if(msg->is_pitch > 0){
                    nextGoal(4) = commandedVelocity/10;
                }

                if(msg->is_yaw < 0){
                    nextGoal(5) = -commandedVelocity/10;
                } else if(msg->is_yaw > 0){
                    nextGoal(5) = commandedVelocity/10;
                }
            }

            nextGoal(3)=0;
            nextGoal(4) = 0;
            nextGoal(5)=0;

            nextDuration = reactionTime;
        }

    }
}