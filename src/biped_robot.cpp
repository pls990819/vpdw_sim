#include "vpdw_sim/biped_robot.h"
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
using namespace std;
namespace Biped
{
    biped_robot::biped_robot(Eigen::Vector2d theta, Eigen::Vector2d vtheta, vector<double> slope, vector<double> slopeLength, vector<double> slopeHeight, double dt, double a, double b, double m, double mb, double g, double J)
    {
        this->_a = a;
        this->_b = b;
        this->_l = a + b;
        this->_m = m;
        this->_mb = mb;
        this->_g = g;
        this->_J = J;
        this->_slope = slope;
        this->_slopeLength = slopeLength;
        this->_slopeHeight = slopeHeight;
        this->_dt = dt;
        this->_theta = theta;
        this->_vtheta = vtheta;
        this->totalSlope = slope.size();
        this->_support_trans = false;
        this->currentSlope = 0;
        this->currentLeftLength = slopeLength[0];
        this->_slope_trans = false;
        Rt << cos(-slope[0]) ,0 ,-sin(-slope[0]),
                  0                 ,1 ,0                 ,
                  sin(-slope[0]) ,0 ,cos(-slope[0]) ;
        Tt << cos(-slope[0]) ,0 ,-sin(-slope[0]),0,
            0               ,1 ,0             ,0,
            sin(-slope[0]) ,0 ,cos(-slope[0]) ,0,
            0             ,0 ,0             ,1;
        Eigen::Matrix<double,4,4> T;
        T << 1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;
        Tt = Tt * T;
    }

    void biped_robot::torqueCallback(std_msgs::Float32MultiArray msg){
        torque << msg.data[0],msg.data[1];
    }

    biped_robot::~biped_robot()
    {
    }

    Eigen::Matrix<double, 2, 2> biped_robot::M(Eigen::Vector2d &theta) const
    {
        Eigen::Matrix<double, 2, 2> _M;
        _M(0, 0) = _J + _m * pow(_a, 2) + _m * pow(_l, 2) + _mb * pow(_l, 2);
        _M(0, 1) = -_m * _l * _b * cos(theta(0) - theta(1));
        _M(1, 0) = _M(0, 1);
        _M(1, 1) = _J + _m * pow(_b, 2);
        return _M;
    }

    Eigen::Matrix<double, 2, 2> biped_robot::C(Eigen::Vector2d &theta, Eigen::Vector2d &vtheta) const
    {
        Eigen::Matrix<double, 2, 2> _C;
        _C(0, 0) = 0;
        _C(0, 1) = -_m * _l * _b * sin(theta(0) - theta(1)) * vtheta(1);
        _C(1, 0) = _m * _l * _b * sin(theta(0) - theta(1)) * vtheta(0);
        _C(1, 1) = 0;
        return _C;
    }

    Eigen::Vector2d biped_robot::G(Eigen::Vector2d &theta) const
    {
        Eigen::Vector2d _G;
        _G(0) = -(_m * _a + _mb * _l + _m * _l) * _g * sin(_slope[currentSlope] + theta(0));
        _G(1) = _m * _g * _b * sin(_slope[currentSlope] + theta(1));
        return _G;
    }

    Eigen::Matrix<double, 2, 2> biped_robot::Q(Eigen::Vector2d &theta) const
    {
        Eigen::Matrix<double, 2, 2> Q1;
        Eigen::Matrix<double, 2, 2> Q2;
        double alpha;
        alpha = theta(0) - theta(1);
        Q1(0, 0) = _J + _m * pow(_a, 2) + _m * pow(_l, 2) + _mb * pow(_l, 2) - _m * _l * _b * cos(alpha);
        Q1(0, 1) = _J + _m * pow(_b, 2) - _m * _l * _b * cos(alpha);
        Q1(1, 0) = -_m * _l * _b * cos(alpha);
        Q1(1, 1) = _J + _m * pow(_b, 2);

        Q2(0, 0) = _J + (2 * _m * _a * _l + _mb * pow(_l, 2)) * cos(alpha) - _m * _a * _b;
        Q2(0, 1) = _J - _m * _a * _b;
        Q2(1, 0) = Q2(0, 1);
        Q2(1, 1) = 0;
        return Q1.inverse() * Q2;
    }

    Eigen::Vector2d biped_robot::getTheta() const
    {
        return _theta;
    }

    Eigen::Vector2d biped_robot::getVtheta() const
    {
        return _vtheta;
    }

    Eigen::Vector4d biped_robot::EulerLagrange(Eigen::Vector4d &&br_state,Eigen::Vector2d &torque)
    {
        Eigen::Vector2d theta, vtheta, vtheta_dot;
        Eigen::Vector4d state_dot;
        theta = br_state.segment(0, 2);
        vtheta = br_state.segment(2, 2);
        vtheta_dot = M(theta).inverse() * (torque - C(theta, vtheta) * vtheta - G(theta));
        state_dot << vtheta, vtheta_dot;
        return state_dot;
    }
    
    void biped_robot::ssPhase()
    {
        // // b
        // double g11 = -(_m * _a + _mb * _l + _m * _l) * _g * sin(_theta(0) + 2*M_PI/180);
        // double g21 = _m * _g * _b * sin(_theta(1) + 2*M_PI/180);
        // Eigen::Vector2d g1(g11,g21);
        // Eigen::Vector2d torque1 = G(_theta) - g1;
        // // e G(angle) - g1
        Eigen::Vector4d state, newstate;
        state << _theta, _vtheta;
        Eigen::Vector4d k1, k2, k3, k4;
        k1 = _dt * EulerLagrange(move(state),torque);
        k2 = _dt * EulerLagrange(state + 0.5 * k1,torque);
        k3 = _dt * EulerLagrange(state + 0.5 * k2,torque);
        k4 = _dt * EulerLagrange(state + k3,torque);
        newstate = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
        _theta = newstate.segment(0, 2);
        _vtheta = newstate.segment(2, 2);
        _support_trans = collision();
    }
    
    void biped_robot::dsPhase()
    {
        double walkingDistanceX = (sin(_theta(0))*_l-sin(_theta(1))*_l);
        double walkingDistanceY = (cos(_theta(0))*_l-cos(_theta(1))*_l);
        Eigen::Matrix<double,4,4> Tstep;
        Tstep << 1,0,0,walkingDistanceX,
                0,1,0,0,
                0,0,1,walkingDistanceY,
                0,0,0,1;
        Tt = Tt * Tstep;
        Eigen::Matrix<double, 2, 2> T;
        Eigen::Vector2d newtheta, newvtheta;
        T << 0, 1, 1, 0;
        newvtheta = Q(_theta) * _vtheta;
        newtheta = T * _theta;
        _theta = newtheta;
        _vtheta = newvtheta;
        if(_slope_trans){
            Eigen::Matrix<double,4,4> Tslopetrans;
            double dSlope = _slope[currentSlope] - _slope[currentSlope-1];
            Tslopetrans << cos(-dSlope), 0,-sin(-dSlope)  , 0 ,
            0 , 1, 0, 0 ,
            sin(-dSlope), 0, cos(-dSlope)  ,0 ,
            0 , 0, 0, 1 ;
            Tt = Tt * Tslopetrans;
            _theta = _theta - Eigen::Vector2d(dSlope,dSlope);
            _slope_trans = false;
        }
        _support_trans = false;
    }

    bool biped_robot::collision()
    {
        double X = _l*sin(_theta(0))-_l*sin(_theta(1));
        double Y = _l*cos(_theta(0))-_l*cos(_theta(1));
        double Y_dot = -_vtheta(0)*_l*sin(_theta(0))+_vtheta(1)*_l*sin(_theta(1));
        if(X > 0 && X < currentLeftLength && Y <= 0 && Y_dot <= 0){ // 未完当前斜坡
            currentLeftLength -= X;
            return true;
        } 
        if(X > currentLeftLength){ // 当前斜坡已结束
            if(currentSlope == totalSlope - 1){
                cout << "finished" << endl;
                exit(0);
                return false;
            }
            double Xnew = X - currentLeftLength;
            Eigen::Matrix<double,3,3> Ta1;
            Ta1 << cos(-_slope[currentSlope]), -sin(-_slope[currentSlope]),  0,
                 sin(-_slope[currentSlope]),  cos(-_slope[currentSlope]),  0,
                 0          ,  0          ,  1;
            Eigen::Matrix<double,3,3> Th;
            Th << 1, 0,  0,
                  0, 1, _slopeHeight[currentSlope+1],
                  0, 0,  1;     
            Eigen::Vector3d p(Xnew,Y,1),pnew;
            Eigen::Matrix<double,3,3> Ta2;
            Ta2 << cos(_slope[currentSlope+1]), -sin(_slope[currentSlope+1]),  0,
                 sin(_slope[currentSlope+1]),  cos(_slope[currentSlope+1]),  0,
                 0          ,  0          ,  1;

            pnew = Ta2*Th*Ta1*p;
            if(pnew(1) <= 0 && Y_dot <= 0){ //这里Y_dot需要重新计算，这么写可能会存在问题，但是主要还是看前面的Y
                ++currentSlope;
                _slope_trans = true;
                currentLeftLength = _slopeLength[currentSlope] - pnew(0);
                return true;
            }
        }
        return false;
    }

    void biped_robot::walking()
    {
        ros::NodeHandle n;
        bipedStatePub = n.advertise<std_msgs::Float32MultiArray>("biped_state", 1000);
        robotPlotPub = n.advertise<visualization_msgs::MarkerArray>("biped_marker",1000);
        torqueSub = n.subscribe("torque", 1000, &Biped::biped_robot::torqueCallback,this);
        ros::Rate loop_rate(100); // 一秒发送100个
        while (ros::ok())
        {   
            for(int i = 0;i < 10;i++){ // 仿真步长0.001s,计算10次发送一次数据
                ssPhase();
                if (_support_trans)
                {
                    dsPhase();
                }
            }
            // 发布机器人状态
            std_msgs::Float32MultiArray bipedState;
            bipedState.data.push_back(_theta(0));
            bipedState.data.push_back(_theta(1));
            bipedState.data.push_back(_vtheta(0));
            bipedState.data.push_back(_vtheta(1));
            bipedStatePub.publish(bipedState);
            plotRobot();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    void biped_robot::showTraj() const
    {
        for (int i = 0; i < _traj.size(); i++)
        {
            cout << _traj[i] << endl;
        }
    }

    void biped_robot::plotRobot(){
        visualization_msgs::MarkerArray markerArray;
        Eigen::Matrix<double,4,4> Ttst; // terrain frame 中描述 stance leg frame
        double t0 = M_PI/2-_theta(0);
        Ttst << cos(t0), 0,-sin(t0), 0 ,
                0      , 1, 0      , 0 ,
                sin(t0), 0, cos(t0), 0 ,
                0      , 0, 0      , 1 ;
        Eigen::Matrix<double,4,4> Tst = Tt*Ttst; // stance leg frame
        Eigen::Vector4d stCenter;
        stCenter = Tst*Eigen::Vector4d(0.5*_l,0,0,1);
        Eigen::Quaterniond quaternionst(Tst.block<3,3>(0,0));
        Eigen::Matrix<double,4,4> Tstsw;
        Eigen::Matrix<double,4,4> Tsto,Tosw;
        Tsto << 1, 0, 0,_l,
                 0 ,1, 0, 0,
                 0 ,0, 1, 0,
                 0 ,0, 0, 1;
        double alpha = -M_PI+_theta(0)-_theta(1);
        Tosw << cos(alpha),0,-sin(alpha), 0,
                0         ,1,0          , 0,
                sin(alpha),0,cos(alpha) , 0,
                0         ,0,0          , 1;
        Tstsw = Tsto * Tosw;
        Eigen::Vector4d swCenter;
        swCenter = Tst*Tstsw*Eigen::Vector4d(0.5*_l,0,0,1);
        Eigen::Matrix<double,4,4> Tsw = Tst*Tstsw;
        Eigen::Quaterniond quaternionsw(Tsw.block<3,3>(0,0));
        for(int i = 0;i < 2;i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = i;
            marker.type =  visualization_msgs::Marker::CUBE;
            marker.action =  visualization_msgs::Marker::ADD;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
            if(i == 0){
                marker.pose.position.x = stCenter(0);
                marker.pose.position.y = stCenter(1);
                marker.pose.position.z = stCenter(2);
                marker.pose.orientation.x = quaternionst.x();
                marker.pose.orientation.y = quaternionst.y();
                marker.pose.orientation.z = quaternionst.z();
                marker.pose.orientation.w = quaternionst.w();
            }else{
                marker.pose.position.x = swCenter(0);
                marker.pose.position.y = swCenter(1);
                marker.pose.position.z = swCenter(2);
                marker.pose.orientation.x = quaternionsw.x();
                marker.pose.orientation.y = quaternionsw.y();
                marker.pose.orientation.z = quaternionsw.z();
                marker.pose.orientation.w = quaternionsw.w();
            }
            marker.scale.x = _l;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            markerArray.markers.push_back(marker);    
        }
        robotPlotPub.publish(markerArray);
    }
}
