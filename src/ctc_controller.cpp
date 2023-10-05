#include<ros/ros.h>
#include<ros/package.h>
#include<Eigen/Eigen>
#include<std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<iostream>
#include<fstream>
#include<string>
using namespace std;

// ROS
ros::Subscriber bipedStateSub;
ros::Subscriber bipedDsSub;
ros::Publisher torquePub;

// controller parameters
Eigen::Matrix<double, 2, 2> K1;
Eigen::Matrix<double, 2, 2> K2;
Eigen::Matrix<double, 2, 2> Kp;
Eigen::Matrix<double, 2, 2> Kd;

// robot and simulation parameters 
double _a = 0.5,_b = 0.5,_l = 1,_m = 5,_mb = 0,_g = 9.8,_J = 0.4167;
double terrainAngle = 2*M_PI/180;
double dt = 0.001;

// robot state and reference trajectory
vector<vector<double>> referenceTrajectory;
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
Eigen::Vector2d torque;
int curt = 0;


// bipedStateSub callback funtion
void bipedStateCallback(std_msgs::Float32MultiArray msg){
    _angle = angle;
    _angularVelocity = angularVelocity;
    angle << msg.data[0],msg.data[1];
    angularVelocity << msg.data[2],msg.data[3];
}

void bipedDsCallback(std_msgs::Bool msg){
    if(msg.data == true){
        finish = false;
        curt = 0;
        cout << "DS" << endl;
    }
}

// Init ROS 
void controllerInit(){
    ros::NodeHandle n;
    bipedStateSub = n.subscribe("biped_state", 1000, bipedStateCallback);
    torquePub = n.advertise<std_msgs::Float32MultiArray>("torque", 1000);
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
    _G(0) = -(_m * _a + _mb * _l + _m * _l) * _g * (sin(theta(0)));
    _G(1) = _m * _g * _b * (sin(theta(1)));
    return _G;
}

// compute torque control
Eigen::Vector2d computeTorque(){
    Eigen::Matrix<double, 2, 2> B;
    B << 1,1,0,1;
    if(!finish){
        return M(angle)*(angularAccelerationRef + Kp * e1 + Kd * e2) + C(angle,angularVelocity) * angularVelocity + G(angle);
    }else{
        double g1 = -(_m * _a + _mb * _l + _m * _l) * _g * sin(angle(0));
        double g2 = _m * _g * _b * sin(angle(1));
        Eigen::Vector2d g(g1,g2);
        return G(angle) - g;
    }
}

// load trajectory
void trajectoryInit(string filename) {
	//filename为读取文件的地址
	ifstream readfile(filename);//打开文件夹
    cout << "loading trajectory" << endl;
	while (!readfile.eof())
	{   
        vector<double> t(6,0);
        for(int i = 0;i < 6;i++){
            readfile >> t[i];
        }
        referenceTrajectory.emplace_back(t);
	}
	readfile.close();//关闭文件夹
    trajectoryLen = referenceTrajectory.size();
    cout << trajectoryLen << endl;
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
            finish = false;
            curt = 0;
            cout << "DS" << endl;
        }
        angleRef << referenceTrajectory[curt][0],referenceTrajectory[curt][1];
        angularVelocityRef << referenceTrajectory[curt][2],referenceTrajectory[curt][3];
        angularAccelerationRef << referenceTrajectory[curt][4],referenceTrajectory[curt][5];  
        // 计算误差
        e1 = angleRef - angle;
        e2 = angularVelocityRef - angularVelocity;
        // 计算输入力矩
        torque = computeTorque();
        // 发布力矩信息
        std_msgs::Float32MultiArray torqueInfo;
        torqueInfo.data.push_back(torque(0));
        torqueInfo.data.push_back(torque(1));
        torquePub.publish(torqueInfo);
        curt+=10; // newtraj是0.001s间隔的点列一秒接收100次，所以一步要加10
        if(curt >= trajectoryLen){
            finish = true;
            curt = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}