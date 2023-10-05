#include<ros/ros.h>
#include<ros/package.h>
#include<Eigen/Eigen>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<iostream>
#include<fstream>
#include<string>
#include "matplotlibcpp.h"
using namespace std;
int nchoosek(int n, int k) {
    if (k == 0 || k == n) {
        return 1;
    }
    return nchoosek(n - 1, k - 1) + nchoosek(n - 1, k);
}

// ROS
ros::Subscriber bipedStateSub;
ros::Publisher torquePub;
ros::Publisher slopePub;
ros::Timer timer;

// controller parameters
Eigen::Matrix<double, 2, 2> K1;
Eigen::Matrix<double, 2, 2> K2;
Eigen::Matrix<double, 2, 2> Kp;
Eigen::Matrix<double, 2, 2> Kd;

// robot and simulation parameters 
double _a = 0.5,_b = 0.5,_l = 1,_m = 5,_mb = 0,_g = 9.8,_J = 0.4167;
double terrainAngle = 2*M_PI/180;
double dt = 0.001;

// estimate parameters
double slope_cos = 1;
double slope_sin = 0;

// robot state and reference trajectory
vector<double> refPSw,refVSw,refASw,refPSt,refVSt,refASt;
vector<double> transPSw,transVSw,transASw,transPSt,transVSt,transASt;
int trajectoryLen;
Eigen::Vector2d angle(0,0);
Eigen::Vector2d angularVelocity(0,0);
Eigen::Vector2d _angle(0,0);
Eigen::Vector2d _angularVelocity(0,0);
Eigen::Vector2d angleRef;
Eigen::Vector2d angularVelocityRef;
Eigen::Vector2d angularAccelerationRef;
Eigen::Vector2d e1,e2;
bool finish = false;
bool replanning = false;
Eigen::Vector2d torque;
int curt = 0;
double zeta = 0.8;
double period;
double connectTime;
int trajectoryStep = 1000;
int transitionTrajectoryLen;

// bipedStateSub callback funtion
void bipedStateCallback(std_msgs::Float32MultiArray msg){
    _angle = angle;
    _angularVelocity = angularVelocity;
    angle << msg.data[0],msg.data[1];
    angularVelocity << msg.data[2],msg.data[3];
}


// Init ROS 
void controllerInit(){
    ros::NodeHandle n;
    bipedStateSub = n.subscribe("biped_state", 1000, bipedStateCallback);
    torquePub = n.advertise<std_msgs::Float32MultiArray>("torque", 1000);
    slopePub = n.advertise<std_msgs::Float32>("slope_estimate", 1000);
    K1 << 5,0,0,5;
    K2 << 12,0,0,12;
    Kp = K1 * K2 + Eigen::MatrixXd::Identity(2,2);
    Kd << K1 + K2;
}

// M Matrix
Eigen::Matrix<double, 2, 2> M(Eigen::Vector2d &theta)
{
    Eigen::Matrix<double, 2, 2> _M;
    _M(0, 0) = _J + _m * pow(_a, 2) + _m * pow(_l, 2) + _mb * pow(_l, 2);
    _M(0, 1) = -_m * _l * _b * cos(theta(0) - theta(1));
    _M(1, 0) = _M(0, 1);
    _M(1, 1) = _J + _m * pow(_b, 2);
    return _M;
}

// C Matrix
Eigen::Matrix<double, 2, 2> C(Eigen::Vector2d &theta, Eigen::Vector2d &vtheta)
{
    Eigen::Matrix<double, 2, 2> _C;
    _C(0, 0) = 0;
    _C(0, 1) = -_m * _l * _b * sin(theta(0) - theta(1)) * vtheta(1);
    _C(1, 0) = _m * _l * _b * sin(theta(0) - theta(1)) * vtheta(0);
    _C(1, 1) = 0;
    return _C;
}

// g vector
Eigen::Vector2d G(Eigen::Vector2d &theta)
{
    Eigen::Vector2d _G;
    _G(0) = -(_m * _a + _mb * _l + _m * _l) * _g * (sin(theta(0)) * slope_cos + cos(theta(0)) * slope_sin);
    _G(1) = _m * _g * _b * (sin(theta(1)) * slope_cos + cos(theta(1)) * slope_sin);
    return _G;
}

// compute torque control
Eigen::Vector2d computeTorque(){
    Eigen::Matrix<double, 2, 2> B;
    B << 1,1,0,1;
    if(!finish){
        return M(angle)*(angularAccelerationRef + Kp * e1 + Kd * e2) + C(angle,angularVelocity) * angularVelocity + G(angle);
    }else{
        double g1 = -(_m * _a + _mb * _l + _m * _l) * _g * sin(angle(0) + terrainAngle);
        double g2 = _m * _g * _b * sin(angle(1) + terrainAngle);
        Eigen::Vector2d g(g1,g2);
        return G(angle) - g;
    }
}

// load trajectory
void trajectoryInit(string filename) {
	//filename为读取文件的地址
	ifstream readfile(filename);//打开文件夹
    cout << "loading trajectory" << endl;
    double t;
	while (!readfile.eof())
	{   
        readfile >> t;
        refPSt.emplace_back(t);
        readfile >> t;
        refPSw.emplace_back(t);
        readfile >> t;
        refVSt.emplace_back(t);
        readfile >> t;
        refVSw.emplace_back(t);
        readfile >> t;
        refASt.emplace_back(t);
        readfile >> t;
        refASw.emplace_back(t);
	}
	readfile.close();//关闭文件夹
    cout << "loading finish" << endl;
    trajectoryLen = refPSt.size();
    cout << trajectoryLen << endl;
    // 初始化重规划参数
    period = (double)trajectoryLen/trajectoryStep;
    connectTime = period * zeta;
    transitionTrajectoryLen = (int)(connectTime*trajectoryStep+1);
    cout << "connectTime" << connectTime << endl;
    cout << "trajectoryStep" << trajectoryStep << endl;
    cout << "transitionTrajectoryLen" << transitionTrajectoryLen << endl;
    transPSw = vector<double>(transitionTrajectoryLen);
    transPSt = vector<double>(transitionTrajectoryLen);
    transVSw = vector<double>(transitionTrajectoryLen);
    transVSt = vector<double>(transitionTrajectoryLen);
    transASw = vector<double>(transitionTrajectoryLen);
    transASt = vector<double>(transitionTrajectoryLen);
}

void parameterUpdate(){
    if(e1(0) < 0.2){
        Eigen::Matrix<double, 2, 2> Minv = M(angle).inverse();
        double g11 = -(_m * _a + _mb * _l + _m * _l) * _g * sin(angle(0));
        double g12 = _m * _g * _b * sin(angle(1));
        double g21 = -(_m * _a + _mb * _l + _m * _l) * _g * cos(angle(0));
        double g22 = _m * _g * _b * cos(angle(1));
        Eigen::Vector2d g1(g11,g12);
        Eigen::Vector2d g2(g21,g22);
        double dslope_cos = (e2+K1*e1).transpose()*(Minv*g1);
        double dslope_sin = (e2+K1*e1).transpose()*(Minv*g2);
        slope_cos += 0.7*dt*dslope_cos;
        slope_sin += 0.7*dt*dslope_sin;
        // slope_cos = 1;
        // slope_sin = 0;
        std_msgs::Float32 angleEstimate;
        angleEstimate.data = asin(slope_sin)*180/M_PI;
        slopePub.publish(angleEstimate);
    }
}

void bezier(vector<double> &&beginState,vector<double> &&endState,double t,vector<double>& p,vector<double>& v,vector<double>& a){
    Eigen::Matrix<double, 6, 6> Aeq;
    double f = pow(t,-1),b = pow(t,-2);
    Aeq    << 1, 0, 0, 0, 0, 0,
         -5*f, 5*f, 0, 0, 0, 0,
       20*b,-40*b,20*b,0, 0, 0,
              0, 0, 0, 0, 0, 1,
         0, 0, 0, 0, -5*f, 5*f,
       0, 0, 0,20*b,-40*b,20*b;
    Eigen::Matrix<double, 6, 1> beq;
    beq << beginState[0], beginState[1], beginState[2],
           endState[0], endState[1], endState[2]; 
    Eigen::Matrix<double, 6, 1> c = Aeq.lu().solve(beq);
    for(int i = 0;i < transitionTrajectoryLen;i++){
        double t1 = (double)i / transitionTrajectoryLen;// 归一化
        double t2 = 1-t1;
        for(int k = 0;k < 6;k++){
            double basis = nchoosek(5,k)*pow(t2,5-k)*pow(t1,k);
            p[i] += c(k)*basis;
        }
        for(int k = 0;k < 5;k++){
            double basis = nchoosek(4,k)*pow(t2,4-k)*pow(t1,k)/t;
            v[i] += 5*(c(k+1)-c(k))*basis;
        }
        for(int k = 0;k < 4;k++){
            double basis = nchoosek(3,k)*pow(t2,3-k)*pow(t1,k)/pow(t,2);
            a[i] += 20*(c(k+2)-2*c(k+1)+c(k))*basis;
        }
    }
    
}

int main(int argc,char** argv){
    ros::init(argc,argv,"control");
    string package_name = "vpdw_sim";
    std::string file_path = ros::package::getPath(package_name) + "/trajectory/newtraj.txt";
    trajectoryInit(file_path);
    controllerInit();
    ros::Rate loop_rate(100);
    while(ros::ok()){
        // 获取机器人状态
        // 获取机器人期望状态
        if(abs(_angle(0)-angle(0)) > 0.03){ // 发生碰撞
            replanning = false;
            finish = false;
            curt = 0;
            cout << "DS" << endl;
            if(abs(_angle(0)-angle(1)) > 1*M_PI/180){
                cout << "estimation update" << endl;
                double phi = asin(slope_sin) + _angle(0) - angle(1);
                slope_sin = sin(phi);
                slope_cos = cos(phi);
            }
            if(!replanning && abs(angle(0)-refPSt[0])+abs(angle(1)-refPSw[0]) > 0.02){
                replanning = true;
                cout << "replanning" << endl;
                fill(transPSw.begin(), transPSw.end(), 0);
                fill(transPSt.begin(), transPSt.end(), 0);
                fill(transVSw.begin(), transVSw.end(), 0);
                fill(transVSt.begin(), transVSt.end(), 0);
                fill(transASw.begin(), transASw.end(), 0);
                fill(transASt.begin(), transASt.end(), 0);
                bezier(vector<double>({angle(0),angularVelocity(0),0}),
                    vector<double>({refPSt[transitionTrajectoryLen-1],refVSt[transitionTrajectoryLen-1],refASt[transitionTrajectoryLen-1]}),connectTime,transPSt,transVSt,transASt);
                bezier(vector<double>({angle(1),angularVelocity(1),0}),
                    vector<double>({refPSw[transitionTrajectoryLen-1],refVSw[transitionTrajectoryLen-1],refASw[transitionTrajectoryLen-1]}),connectTime,transPSw,transVSw,transASw);
            }
        }
        if(!finish){
            if(!replanning || curt >= transitionTrajectoryLen){
                angleRef << refPSt[curt],refPSw[curt];
                angularVelocityRef << refVSt[curt],refVSw[curt];
                angularAccelerationRef << refASt[curt],refASw[curt];
            }else{
                angleRef << transPSt[curt],transPSw[curt];
                angularVelocityRef << transVSt[curt],transVSw[curt];
                angularAccelerationRef << transASt[curt],transASw[curt];
            }
            // 计算误差
            e1 = angleRef - angle;
            e2 = angularVelocityRef - angularVelocity;
            // 估计参数更新
            parameterUpdate();
        }       
        // 计算输入力矩
        torque = computeTorque();
        // 发布力矩信息
        std_msgs::Float32MultiArray torqueInfo;
        torqueInfo.data.push_back(torque(0));
        torqueInfo.data.push_back(torque(1));
        torquePub.publish(torqueInfo);
        if(!finish){
            curt+=10;
            if(curt >= trajectoryLen){
                finish = true;
                curt = 0;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}