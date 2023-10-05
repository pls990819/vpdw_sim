#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include "matplotlibcpp.h"

using namespace std;

namespace plt = matplotlibcpp;

ros::Timer timer;
ros::Subscriber bipedStateSub;
ros::Subscriber slopeEstimateSub;

int i = 0 ;
vector<double> PSt(0),PSw(0),VSt(0),VSw(0),slopeEstimate(0),t(0);
double _st = 0,st = 0;

void bipedStateCallback(std_msgs::Float32MultiArray msg){
    _st = st;
    st = msg.data[0];
    // if(abs(st - _st) > 0.03){
    //     PSt.clear();
    //     PSw.clear();
    //     VSt.clear();
    //     VSw.clear();
    // }
    PSt.emplace_back(msg.data[0]);
    PSw.emplace_back(msg.data[1]);
    VSt.emplace_back(msg.data[2]);
    VSw.emplace_back(msg.data[3]);
}

void slopeEstimateCallback(std_msgs::Float32 msg){
    slopeEstimate.emplace_back(msg.data);
    t.emplace_back((i++)*0.01);
}

void updatePlot(const ros::TimerEvent &){
    plt::clf();  //加这句为了动态画图
    // plt::plot(refPSw,refVSw,"g--");
    // plt::plot(refPSt,refVSt,"b--");
    plt::subplot(2,1,1);
    plt::plot(PSt,VSt,"r-");
    plt::plot(PSw,VSw,"b-");
    plt::title("Walking Phase");
    plt::xlim(-1,1);
    plt::ylim(-4,4);
    plt::subplot(2,1,2);
    plt::plot(t,slopeEstimate,"r-");
    plt::title("Slope Estimate");
    plt::xlim(0,20);
    plt::grid(true);
    // plt::ylim(-10,10);
    plt::pause(0.01);
}

void pltInit(){
    ros::NodeHandle n;
    bipedStateSub = n.subscribe("biped_state", 10, bipedStateCallback);
    slopeEstimateSub = n.subscribe("slope_estimate",10, slopeEstimateCallback);
    timer = n.createTimer(ros::Duration(0.01),updatePlot);
}

int main(int argc,char** argv){
    ros::init(argc,argv,"plt");
    pltInit();
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}