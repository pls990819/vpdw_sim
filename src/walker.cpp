#include "vpdw_sim/biped_robot.h"
#include <vector>
using namespace Biped;
int main(int argc, char **argv)
{
    double a,b,m,mb,g,J;
    ros::init(argc, argv, "biped_walking_node");    
    std::vector<double> initState, slopeA, slopeL, slopeH;
    ros::param::get("a", a);
    ros::param::get("b", b);
    ros::param::get("m", m);
    ros::param::get("mb", mb);
    ros::param::get("g", g);
    ros::param::get("J", J);
    ros::param::get("init_state", initState); 
    ros::param::get("slopeAngle",slopeA);
    ros::param::get("slopeLength",slopeL);
    ros::param::get("slopeHeight",slopeH);
    Eigen::Vector2d theta(initState[0], initState[1]);
    Eigen::Vector2d vtheta(initState[2], initState[3]);
    for(int i = 0;i < slopeA.size();i++){
        slopeA[i] = slopeA[i] * M_PI /180;
    }
    biped_robot br(theta, vtheta, slopeA, slopeL, slopeH,0.001,a,b,m,mb,g,J);
    br.walking();
    return 0;
}
