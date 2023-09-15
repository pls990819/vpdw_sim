#include <ros/ros.h>
#include <Eigen/Eigen>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using namespace std;
Eigen::Vector2d angle;
Eigen::Vector2d angularVelocity;
Eigen::Vector2d _angle;
Eigen::Vector2d _angularVelocity;
ros::Subscriber bipedStateSub;
ros::Publisher robotPlotPub;
ros::Publisher terrainPlotPub;
Eigen::Matrix<double,3,3> Rt;
double terrainAngle = -2*M_PI/180;
Eigen::Matrix<double,4,4> Tt;
double length = 1.0;
std::vector<double> slopeAngle, slopeLength, slopeHeight;

void bipedStateCallback(std_msgs::Float32MultiArray msg){
    _angle = angle;
    _angularVelocity = angularVelocity;
    angle << msg.data[0],msg.data[1];
    angularVelocity << msg.data[2],msg.data[3];
}


void plotTerrain(){
    visualization_msgs::MarkerArray markerArray;
    Eigen::Vector4d slopeCenter;
    double angle = 0;
    double len = 0;
    Eigen::Matrix<double,4,4> Tslope = Eigen::MatrixXd::Identity(4, 4);
    for(int i = 0;i < slopeAngle.size();i++){
        Eigen::Matrix<double,4,4> Ta; // 角度变化导致的齐次变换矩阵
        Ta << cos(slopeAngle[i] - angle) ,0 ,-sin(slopeAngle[i] - angle),len,
              0                          ,1 ,0                          ,0,
              sin(slopeAngle[i] - angle) ,0 ,cos(slopeAngle[i] - angle) ,0,
              0                          ,0 ,0                          ,1;
        Eigen::Matrix<double,4,4> Th; // 高度变化导致的齐次变换矩阵
        Th << 1,0,0,0,
              0,1,0,0,
              0,0,1,-slopeHeight[i],
              0,0,0,1;
        Tslope = Th*Tslope*Ta; // 高度相对于固定坐标所以左乘，角度相对于变化坐标系所以右乘
        slopeCenter = Tslope*Eigen::Vector4d(0.5*slopeLength[i],0,0,1); // 计算该斜坡中心坐标
        angle = slopeAngle[i];
        len = slopeLength[i];
        Eigen::Quaterniond quaternionslope(Tslope.block<3,3>(0,0)); // 计算斜坡方向角四元数
        visualization_msgs::Marker marker; // 生成斜坡topic，数据结构为MarkerArray
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type =  visualization_msgs::Marker::CUBE;
        marker.action =  visualization_msgs::Marker::ADD;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1;
        marker.pose.position.x = slopeCenter(0);
        marker.pose.position.y = slopeCenter(1);
        marker.pose.position.z = slopeCenter(2);
        marker.pose.orientation.x = quaternionslope.x();
        marker.pose.orientation.y = quaternionslope.y();
        marker.pose.orientation.z = quaternionslope.z();
        marker.pose.orientation.w = quaternionslope.w();
        marker.scale.x = slopeLength[i];
        marker.scale.y = 0.1;
        marker.scale.z = 0.001;
        markerArray.markers.push_back(marker); 
    }
    terrainPlotPub.publish(markerArray); // 发布topic
}


void plot(){
    ros::NodeHandle n;
    bipedStateSub = n.subscribe("biped_state", 1000, bipedStateCallback);
    terrainPlotPub = n.advertise<visualization_msgs::MarkerArray>("terrain_marker",1000);
    ros::Rate loop_rate(100);
    while(ros::ok()){
        plotTerrain();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc,char **argv){
    ros::init(argc,argv,"plot_walking");
    ros::param::get("slopeAngle",slopeAngle);
    ros::param::get("slopeLength",slopeLength);
    ros::param::get("slopeHeight",slopeHeight);
    for(int i = 0;i < slopeAngle.size();i++){
        slopeAngle[i] = - slopeAngle[i] * M_PI /180;
    }
    // initT();// 初始化斜面R和T
    plot();
    return 0;
}
