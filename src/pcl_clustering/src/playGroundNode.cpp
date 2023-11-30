//
// Created by areza on 11/30/23.
//


// Note: This file is overwritten periodically. Don't leave important data on it.

#include <ros/ros.h>
#include <Eigen/Dense>

using Eigen::VectorXf;
using Eigen::MatrixXf;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "playGroundNode");
    ros::NodeHandle nh;
    MatrixXf H(3, 6);
    H.setIdentity();
    std::cout << "H is: " << std::endl << H << std::endl << "--------------" << std::endl;
    VectorXf Y(3);
    Y.setOnes();
    std::cout << "Y is: " << std::endl << Y << std::endl << "--------------" << std::endl;
    VectorXf X(6);
    X = 2 * X.setOnes();
    std::cout << "X is: " << std::endl << X << std::endl << "--------------" << std::endl;

    auto result = Y - H * X;
    std::cout << "Result is: " << std::endl << result << std::endl << "--------------" << std::endl;

    VectorXf hassani;
    const VectorXf &kelid = X;
    hassani = kelid;
    std::cout << "hassani is: " << std::endl << hassani << std::endl << "--------------" << std::endl;

    return 0;
}