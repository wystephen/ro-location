#pragma once
//Create by steve in 16-10-21 at 下午3:44
//
// Created by steve on 16-10-21.
//


#include <deque>
#include <vector>

#include <Eigen/Dense>

#include <random>

#include <chrono>

#include <ctime>

#include "../Cpp_Extent/MyError.h"


#ifndef RO_LOCATION_VIRTUALODOMETRY_H
#define RO_LOCATION_VIRTUALODOMETRY_H
namespace OPF {
    class VirtualOdometry {
    public:

        VirtualOdometry(Eigen::MatrixXd b_pose, double z_offset) {
            z_offset_ = z_offset;
            beacon_pose_ = b_pose;
        }

        Eigen::VectorXd odometry(Eigen::VectorXd position, Eigen::VectorXd range) {
            Eigen::Vector2d out_pose(position(0), position(1));
            double max_score(0.0);


            for (int i(-30); i < 30; ++i) {
                for (int j(-30); j < 30; ++j) {
                    double score = GetScore(Eigen::VectorXd(Eigen::Vector2d(position(0) + double(i) / 30.0 * 0.5,
                                                                            position(1) + double(j) / 30.0 * 0.5)),
                                            range);
                    if (score > max_score) {
                        max_score = score;
                        out_pose(0) = double(double(i) / 30.0 * 0.5);
                        out_pose(1) = double(double(j) / 30.0 * 0.5);
                    }

                }
            }
            return out_pose;
        }

        double GetScore(Eigen::VectorXd guess_state, Eigen::VectorXd range_vec) {
            std::normal_distribution<> n(0, 0.012);

            double ret(0.0);
            Eigen::Vector3d dis;

            for (int i(0); i < 3; ++i) {
                dis(i) = 0.0;
                dis(i) += std::pow(guess_state(0) - beacon_pose_(i, 0), 2.0);
                dis(i) += std::pow(guess_state(1) - beacon_pose_(i, 1), 2.0);
                dis(i) += std::pow(z_offset_ - beacon_pose_(i, 2), 2.0);
                dis(i) = std::pow(dis(i), 0.5);
//            ret += 1 / std::sqrt(2 * M_PI) / sigma_(i + 2.0) * std::exp(
//                    -std::pow(dis(i) - (range_vec(i)), 2.0) / 2.0 /
//                    std::pow(sigma_(i + 2.0), 2.0));
                //ret *= (NormalPDF(range_vec(i), dis(i) + n(e_), 0.08)+1e-8) ;//* NormalPDF(range_vec(i), avg_range_(i), 0.4);
                /*
                 * bias
                 */
                auto f = [=] {
                    return 0.1 * (1.01 - std::exp(-0.17 * dis(i)));
                };
//            /*
//             * randon bias
//             */
//            auto b = [=] {
//                std::uniform_real_distribution<double> rnd(0, 1);
//                if (rnd(e_) < 0.05) {
//                    return guess_state(2_i) + (rnd(e_)-0.5) * 10.0;
//                } else {
//
//                }
//            };
//            b();
                ret += LogNormalPDF(range_vec(i), dis(i) + f(), 0.08);

            }


//        return std::pow(2.0,ret);
            return (ret);
        }

        double LogNormalPDF(double x, double miu, double sigma) {
            return std::log(1 / std::sqrt(2.0 * M_PI) / sigma) / (std::pow(x - miu, 2.0) / 2 / sigma / sigma);
        }

        Eigen::MatrixXd beacon_pose_;//every row represts a beacon's position.

        double z_offset_;//offset of the tag at axis z.




    protected:

    private:


    };
}

#endif //RO_LOCATION_VIRTUALODOMETRY_H
