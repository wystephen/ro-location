//Create by steve in 16-10-14 at 上午9:38
//
// Created by steve on 16-10-14.
//

#include <Eigen/Dense>

#include <iostream>

#include "../include/Cpp_Extent/matplotlib_interface.h"

#include <random>

#include <cmath>

double TwoDnormal(double x,
                  double y,
                  double miu1,
                  double miu2,
                  double rho,
                  double sigma1,
                  double sigma2) {

    if (miu1 > 1000) {
        return 0.0000000001;
    }

    double para1, para2;
    para1 = 2.0 * M_PI * sigma1 * sigma2 * std::pow(1.0 - rho * rho, 0.5);

    para2 = -1.0 / 2.0 / (1.0 - rho * rho) *
            (std::pow(x - miu1, 2.0) / sigma1 / sigma1 + 2.0 * rho * (x - miu1) * (y - miu2) / sigma1 / sigma2
             + std::pow(y - miu2, 2.0) / sigma2 / sigma2);


    return 1 / para1 * std::exp(para2);
}


int main() {
    std::vector<double> dx, dy;
    std::cout.precision(29);
//    for (int i(-1000); i < 1000; ++i) {
//        for (int j(-1000); j < 1000; ++j) {
//            if (i == 50) {
//                std::cout << double(i) / 300 << " " << double(j) / 300 << " "
//                          << TwoDnormal(double(i) / 300.0, double(j) / 300.0, 0.0, 0.0, 0.0, 0.2, 0.2)
//                          << std::endl;
//                dx.push_back(double(j) / 300.0);
//                dy.push_back(TwoDnormal(double(i) / 300.0, double(j) / 300.0, 0.0, 0.0, 0.0, 0.2, 0.2));
//            }
//
//        }
//    }

    std::default_random_engine e_;//Random generator engine.

    std::normal_distribution<> nor_get(0.1, 0.03);
    std::uniform_real_distribution<double> angle_get(-M_PI, M_PI);


    for (int j(0); j < 10000; ++j) {
        double dis(nor_get(e_));
        double angle(angle_get(e_));
        double delta_x(dis * std::cos(angle));
        double delta_y(dis * std::sin(angle));
        std::cout << "sin:" << std::sin(angle) << " cos:" << std::cos(angle) << std::endl;

        dx.push_back((delta_x));
        dy.push_back(delta_y);
    }

    matplotlibcpp::plot(dx, dy, "r+");
    matplotlibcpp::show();


    return 0;
}