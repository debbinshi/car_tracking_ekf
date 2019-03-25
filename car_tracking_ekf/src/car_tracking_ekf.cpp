#include "car_tracking_ekf.h"

car_tracking_ekf::car_tracking_ekf(double T){
    
    T_ = T;
    
    H_ = Eigen::MatrixXd::Zero(3, 9);
    H_ << 1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 1, 0, 0;
#ifdef constant_acceleration
    fai_ = Eigen::MatrixXd::Zero(9, 9);
    fai_ << 1, T_, 0, 0, 0, 0, 0, 0, 0,
            0, 1, T_, 0, 0, 0, 0, 0, 0,
            0, 0,  1, 0, 0, 0, 0, 0, 0,
            0, 0,  0, 1, T_,0, 0, 0, 0,
            0, 0,  0, 0, 1, T_,0, 0, 0,
            0, 0,  0, 0, 0, 1, 0, 0, 0,
            0, 0,  0, 0, 0, 0, 1, T_, 0,
            0, 0,  0, 0, 0, 0, 0, 1, T_,
            0, 0,  0, 0, 0, 0, 0, 0, 1;

    double kesi_ax = 0.001;
    double kesi_ay = 0.001;
    double kesi_az = 0.001;

    Q_ = Eigen::MatrixXd::Zero(9, 9);
    Q_ << 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, kesi_ax * kesi_ax, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, kesi_ay * kesi_ay, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, kesi_az * kesi_az;
#endif
#ifdef constant_velocity
    fai_ = Eigen::MatrixXd::Zero(9, 9);
    fai_ << 1, T_, 0, 0, 0, 0, 0, 0, 0,
            0, 1,  0, 0, 0, 0, 0, 0, 0,
            0, 0,  1, 0, 0, 0, 0, 0, 0,
            0, 0,  0, 1, T_,0, 0, 0, 0,
            0, 0,  0, 0, 1, 0, 0, 0, 0,
            0, 0,  0, 0, 0, 1, 0, 0, 0,
            0, 0,  0, 0, 0, 0, 1, T_,0,
            0, 0,  0, 0, 0, 0, 0, 1, 0,
            0, 0,  0, 0, 0, 0, 0, 0, 1;

    double kesi_vx = 0.1;
    double kesi_vy = 0.1;
    double kesi_vz = 0.1;
    
    Q_ = Eigen::MatrixXd::Zero(9, 9);
    Q_ << 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, kesi_vx * kesi_vx, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, kesi_vy * kesi_vy, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, kesi_vz * kesi_vz, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0;
#endif

    double kesi_x = 1.0;
    double kesi_y = 1.0;
    double kesi_z = 1.0;
    
    R_= Eigen::MatrixXd::Zero(3, 3);
    R_ << kesi_x * kesi_x, 0, 0,
          0, kesi_y * kesi_y, 0,
          0, 0, kesi_z * kesi_z;
   
    esti_car_pose_ = Eigen::VectorXd::Zero(9, 1);
    esti_car_covariance_ = 10 * Eigen::MatrixXd::Identity(9, 9);

    raw_car_pose_x_ = 0.0;
    raw_car_pose_y_ = 0.0;
    raw_car_pose_z_ = 0.0;

    update_ = true;                                     
}

void car_tracking_ekf::car_tracking_process(const ros::TimerEvent& event){
    update(raw_car_pose_x_, raw_car_pose_y_, raw_car_pose_z_);
}

void car_tracking_ekf::update(double raw_car_pose_x, double raw_car_pose_y, double raw_car_pose_z){

    if(update_ == false){
        Eigen::VectorXd raw_car_pose(3);
        raw_car_pose << raw_car_pose_x, raw_car_pose_y, raw_car_pose_z;
    
//state prediction
        Eigen::VectorXd predict_car_pose = fai_ * esti_car_pose_;
        Eigen::MatrixXd predict_car_covariance = fai_ * esti_car_covariance_ * fai_.transpose() + Q_;
//innovation
        Eigen::VectorXd innovation_car_pose = raw_car_pose - H_ * predict_car_pose;
//Kalman gain calculation
        Eigen::MatrixXd K = predict_car_covariance * H_.transpose() * ((H_ * predict_car_covariance * H_.transpose() + R_).inverse());        

//state estimate
        esti_car_pose_ = predict_car_pose + K * innovation_car_pose;
        esti_car_covariance_ = (Eigen::MatrixXd::Identity(9, 9) - K * H_) * predict_car_covariance;

        update_ = true;
    } 
}

bool car_tracking_ekf::get_update_state(){
    return update_;
}

void car_tracking_ekf::change_update_state(bool update){
    update_ = update;
}

void car_tracking_ekf::get_estimate_car_pose(vector<double>& esti_car_pose){
    esti_car_pose[0] = esti_car_pose_(0);
    esti_car_pose[1] = esti_car_pose_(1);
    esti_car_pose[2] = esti_car_pose_(2);
    esti_car_pose[3] = esti_car_pose_(3);
    esti_car_pose[4] = esti_car_pose_(4);
    esti_car_pose[5] = esti_car_pose_(5);
    esti_car_pose[6] = esti_car_pose_(6);
    esti_car_pose[7] = esti_car_pose_(7);
    esti_car_pose[8] = esti_car_pose_(8);
}

void car_tracking_ekf::change_car_pose(double car_pose_x, double car_pose_y, double car_pose_z){
    raw_car_pose_x_ = car_pose_x;
    raw_car_pose_y_ = car_pose_y;
    raw_car_pose_z_ = car_pose_z;
}

car_tracking_ekf::~car_tracking_ekf(){}

