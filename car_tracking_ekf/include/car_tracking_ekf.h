#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "ros/ros.h"
using namespace std;

//#define constant_velocity
#define constant_acceleration

class car_tracking_ekf{
public:
    car_tracking_ekf(double T); 
    ~car_tracking_ekf();
    void car_tracking_process(const ros::TimerEvent& event);
    void update(double raw_car_pose_x, double raw_car_pose_y, double raw_car_pose_z);

    bool get_update_state();
    void change_update_state(bool update);

    void get_estimate_car_pose(vector<double>& esti_car_pose);
    void change_car_pose(double car_pose_x, double car_pose_y, double car_pose_z);
private:
    double T_;

    Eigen::MatrixXd H_;
    Eigen::MatrixXd fai_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

    Eigen::VectorXd esti_car_pose_;
    Eigen::MatrixXd esti_car_covariance_;            

    double raw_car_pose_x_;
    double raw_car_pose_y_;
    double raw_car_pose_z_;

    bool update_;
};
