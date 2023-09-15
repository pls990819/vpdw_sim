#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
using namespace std;
namespace Biped
{
    class biped_robot
    {
    private:
        double _a;
        double _b;
        double _l;
        double _m;
        double _mb;
        double _g;
        double _J;
        vector<double> _slope;
        vector<double> _slopeLength;
        vector<double> _slopeHeight;
        Eigen::Vector2d _theta;
        Eigen::Vector2d _vtheta;
        bool _support_trans;
        bool _slope_trans;
        int currentSlope;
        int totalSlope;
        double currentLeftLength;
        double _dt;
        vector<Eigen::Vector4d> _traj;
        Eigen::Matrix<double,4,4> Tt;
        Eigen::Matrix<double,3,3> Rt;
        ros::Publisher robotPlotPub;
        ros::Publisher bipedStatePub;
        ros::Subscriber torqueSub;
        Eigen::Vector2d torque;

    public:
        biped_robot(Eigen::Vector2d theta,
                    Eigen::Vector2d vtheta,
                    vector<double> slope,
                    vector<double> slopeLength,
                    vector<double> slopeHeight,
                    double dt = 0.001,
                    double a = 0.5,
                    double b = 0.5,
                    double m = 5,
                    double mb = 0,
                    double g = 9.8,
                    double J = 0.4167);
        ~biped_robot();
        void torqueCallback(std_msgs::Float32MultiArray msg);
        Eigen::Matrix<double, 2, 2> M(Eigen::Vector2d &) const;
        Eigen::Matrix<double, 2, 2> C(Eigen::Vector2d &, Eigen::Vector2d &) const;
        Eigen::Vector2d G(Eigen::Vector2d &) const;
        Eigen::Vector2d getTheta() const;
        Eigen::Vector2d getVtheta() const;
        Eigen::Matrix<double, 2, 2> Q(Eigen::Vector2d &) const;
        Eigen::Vector4d EulerLagrange(Eigen::Vector4d &&,Eigen::Vector2d &);
        void ssPhase();
        void dsPhase();
        bool collision();
        void walking();
        void showTraj() const;
        void plotRobot();
    };
    // Eigen::Vector4d LungeKutta(Eigen::Vector4d (*fun)(Eigen::Vector4d), Eigen::Vector4d &, double);
}
