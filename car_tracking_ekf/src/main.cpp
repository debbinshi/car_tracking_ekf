#include "car_tracking_ekf.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/ros.h"
#include "math.h"
#include <fstream>
using namespace std;

double T_ekf = 0.1;
double Ts = 0.01;
double t_total = 10.0;

vector<vector<double>> raw_car_pose;
vector<vector<double>> real_car_pose;
double gauss_rand(double kesi){
    static double V1, V2, S;
    static int phase = 0;
    double X;

    if(phase == 0){
        do{
            double U1 = (double)rand() / RAND_MAX;
            double U2 = (double)rand() / RAND_MAX;
            
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        }while(S >= 1 || S == 0);
        X = V1 * sqrt(-2 * log(S) / S);
    }
    else{
        X = V2 * sqrt(-2 * log(S) / S);
    }

    phase = 1 - phase;

    return X * kesi * kesi;
}

void generate_data(double Ts, double t_total){
    Eigen::VectorXd w(9);
    Eigen::VectorXd v(3);
    Eigen::VectorXd X(9);
    Eigen::VectorXd Y(3);
     
#ifdef constant_velocity
    double kesi_vx = 0.1;
    double kesi_vy = 0.1;
    double kesi_vz = 0.1;
   
    X(0) = 0.0;
    X(1) = 1.0;
    X(2) = 0.0;
    X(3) = 0.0;
    X(4) = 0.5;
    X(5) = 0.0;
    X(6) = 0.0;
    X(7) = 0.0;
    X(8) = 0.0;
 
    Eigen::MatrixXd fai(9, 9);
    fai << 1, Ts, 0, 0, 0, 0, 0, 0, 0,
           0, 1,  0, 0, 0, 0, 0, 0, 0,
           0, 0,  1, 0, 0, 0, 0, 0, 0,
           0, 0,  0, 1, Ts,0, 0, 0, 0,
           0, 0,  0, 0, 1, 0, 0, 0, 0,
           0, 0,  0, 0, 0, 1, 0, 0, 0,
           0, 0,  0, 0, 0, 0, 1, Ts, 0,
           0, 0,  0, 0, 0, 0, 0, 1, 0,
           0, 0,  0, 0, 0, 0, 0, 0, 1;
#endif
#ifdef constant_acceleration
    double kesi_ax = 0.001;
    double kesi_ay = 0.001;
    double kesi_az = 0.001;

    X(0) = 0.0;
    X(1) = 0.0;
    X(2) = 0.1;
    X(3) = 0.0;
    X(4) = 0.0;
    X(5) = 0.1;
    X(6) = 0.0;
    X(7) = 0.0;
    X(8) = 0.0;

    Eigen::MatrixXd fai(9, 9);
    fai << 1, Ts, 0, 0, 0, 0, 0, 0, 0,
           0, 1, Ts, 0, 0, 0, 0, 0, 0,
           0, 0,  1, 0, 0, 0, 0, 0, 0,
           0, 0,  0, 1, Ts,0, 0, 0, 0,
           0, 0,  0, 0, 1, Ts,0, 0, 0,
           0, 0,  0, 0, 0, 1, 0, 0, 0,
           0, 0,  0, 0, 0, 0, 1, Ts, 0,
           0, 0,  0, 0, 0, 0, 0, 1, Ts,
           0, 0,  0, 0, 0, 0, 0, 0, 1;
#endif
    double kesi_x = 1.0;
    double kesi_y = 1.0;
    double kesi_z = 1.0;

    Eigen::MatrixXd H(3, 9);
                           H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 1, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 1, 0, 0;

    for(int i = 0; i < t_total / Ts; i ++){
#ifdef constant_velocity
        w << 0, gauss_rand(kesi_vx), 0, 0, gauss_rand(kesi_vy), 0, 0, 0, gauss_rand(kesi_vz); 
#endif
#ifdef constant_acceleration
        w << 0, 0, gauss_rand(kesi_ax), 0, 0, gauss_rand(kesi_ay), 0, 0, gauss_rand(kesi_az); 
#endif
         
        X = fai * X + w;
        v << gauss_rand(kesi_x), gauss_rand(kesi_y), gauss_rand(kesi_z);
        Y = H * X + v;
        
        vector<double> car_pose1{Y(0), Y(1), Y(2)};
        raw_car_pose.push_back(car_pose1);   
        
        vector<double> car_pose2{X(0), X(1), X(2), X(3), X(4), X(5), X(6), X(7), X(8)};
        real_car_pose.push_back(car_pose2);   
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "car_tracking");

    ros::NodeHandle n;
    ros::Timer timer;
    
    generate_data(Ts, t_total);
    
    car_tracking_ekf fusioner(T_ekf);
    timer = n.createTimer(ros::Duration(T_ekf), &car_tracking_ekf::car_tracking_process, &fusioner);

    ros::Rate loop_rate(1.0 / Ts);
    
    int count = 0;
    vector<double> esti_car_pose;
    esti_car_pose.resize(9);
    
    ofstream out;
    ofstream out_raw;
    ofstream out_real;
    if(!out.is_open()){
        out.open("esti_pose.txt");
    }
    if(!out_raw.is_open()){
        out_raw.open("raw_pose.txt");
    }
    if(!out_real.is_open()){
        out_real.open("real_pose.txt");
    }

    cout << "filtering!" << endl;
    while(ros::ok()){
        if(count == (t_total / Ts - 1))
            break;
        ros::spinOnce();

        fusioner.change_car_pose(raw_car_pose[count][0], raw_car_pose[count][1], raw_car_pose[count][2]);
        count++;
    
        out_raw << raw_car_pose[count][0] << " " << raw_car_pose[count][1] << " " << raw_car_pose[count][2] << endl;
        out_real << real_car_pose[count][0] << " " << real_car_pose[count][1] << " " << real_car_pose[count][2] << " "<<
                    real_car_pose[count][3] << " " << real_car_pose[count][4] << " " << real_car_pose[count][5] << " "<<
                    real_car_pose[count][6] << " " << real_car_pose[count][7] << " " << real_car_pose[count][8] << endl;

        if(fusioner.get_update_state() == true){
            fusioner.get_estimate_car_pose(esti_car_pose);
            fusioner.change_update_state(false);
            
            out << esti_car_pose[0] << " " << esti_car_pose[1] << " " << esti_car_pose[2] << " "<<
                   esti_car_pose[3] << " " << esti_car_pose[4] << " " << esti_car_pose[5] << " "<<
                   esti_car_pose[6] << " " << esti_car_pose[7] << " " << esti_car_pose[8] << endl;
        }
        loop_rate.sleep();
    }
    
    cout << "end filter!" << endl;
    
    out.close();
    out_raw.close();
    out_real.close();

    return 1;
}


