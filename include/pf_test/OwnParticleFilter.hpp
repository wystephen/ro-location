#pragma once
//
// Created by steve on 16-10-13.
//

//Create by steve in 16-10-13 at 上午9:50

#include <deque>
#include <vector>

#include <Eigen/Dense>

#include <random>

#include <chrono>

#include <ctime>

#include "../Cpp_Extent/MyError.h"


namespace OPF {
    class OwnParticleFilter {
    public:

        OwnParticleFilter(long p_num, Eigen::MatrixXd b_pose, double z_offset_value, long history_len) {
            particle_num_ = p_num;
            beacon_pose_ = b_pose;
            z_offset_ = z_offset_value;

            history_len_ = history_len;

            e_.seed(time(NULL));

            state_.resize(2 + beacon_pose_.rows());

            sigma_.resize(state_.rows());

            for (int i(0); i < sigma_.rows(); ++i) {
                sigma_(i) = 0.15;
            }


        }

        bool InitialState(Eigen::Vector2d real_pose);

        bool Sample();

        bool Evaluate(Eigen::VectorXd range_vec);

        double Likelihood(Eigen::VectorXd guess_state, Eigen::VectorXd range_vec);

        Eigen::VectorXd GetResult();

        bool ReSample();

        bool ComputeCPoint(Eigen::VectorXd range);


    protected:

        long particle_num_ = 1000;//particle number.

//        std::deque<Eigen::VectorXd> range_history_;//save a sequence of range.

        std::deque<Eigen::VectorXd> state_history_;//save a sequence of state.

        Eigen::VectorXd state_;//Current state vec

        Eigen::MatrixXd beacon_pose_;//every row represts a beacon's position.

        double z_offset_;//offset of tag at axis z.

        long history_len_ = 10;//the size of state_history deque.

        Eigen::MatrixXd particle_mx_;//The matrix saved the particle

        Eigen::VectorXd weight_vec_;//The vector saved the particle

        std::default_random_engine e_;//Random generetor engine.

        Eigen::VectorXd sigma_;//sigma of state_

        Eigen::Matrix<double, 6, 2> con_point_;


    private:
        /*
         *
         */
        Eigen::Vector3d Two2Three(Eigen::Vector2d pose) {

            return Eigen::Vector3d(pose(0), pose(1), z_offset_);
        }

        double TwoDnormal(double x, double y, double miu1, double miu2, double rho, double sigma1, double sigma2);


    };

    double OwnParticleFilter::TwoDnormal(double x,
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

    bool OwnParticleFilter::ComputeCPoint(Eigen::VectorXd range) {
        /*
         * Step 1: convert range in 3d to range_2d.
         */
        Eigen::VectorXd range2d(range);

        for (int i(0); i < range.size(); ++i) {
            range2d(i) = std::pow((range(i) * range(i) - std::pow(beacon_pose_(i, 2) - z_offset_, 2)), 0.5);
        }

        /*
         * Step 2: Compute beacon to beacon 2d distance
         */

        Eigen::MatrixXd dis_mat(range.rows(), range.rows());
        for (int i(0); i < range.rows(); ++i) {
            for (int j(0); j < range.rows(); ++j) {
                if (i == j) {
                    continue;
                } else {
                    dis_mat(i, j) = std::pow(std::pow(beacon_pose_(i, 0) - beacon_pose_(j, 0), 2) +
                                             std::pow(beacon_pose_(i, 1) - beacon_pose_(j, 1), 2), 0.5);
                    dis_mat(j, i) = dis_mat(i, j);
                }
            }
        }

//        std::cout <<"dis mat :" << dis_mat << std::endl;
//        std::cout << "range 2d :" << range2d.transpose() << std::endl;

        /*
         * Step 3: Compute common points
         */


        int index(0);

        for (int i(0); i < range.rows(); ++i) {
            for (int j(0); j < i; ++j) {
                if (range2d(j) + range2d(i) < dis_mat(i, j) * 0.98) {
                    //without common point
                    con_point_(index, 0) = 111111111;
                    con_point_(index, 1) = 111111111;
                    ++index;
                    con_point_(index, 0) = 111111111;
                    con_point_(index, 1) = 111111111;
                    ++index;
                } else {
                    double the_angle = range2d(j) / (range2d(j) + range2d(i) + dis_mat(i, j)) * M_PI;

                    double offsetx(0.0), offsety(0.0),
                            x2(0.0), y2(0.0),
                            x2n(0.0), y2n(0.0),
                            x3n(0.0), x3(0.0),
                            y3n(0.0), y3(0.0);
                    double l1 = range2d(i);

                    offsetx = beacon_pose_(i, 0);
                    offsety = beacon_pose_(i, 1);

                    x2 = beacon_pose_(j, 0) - beacon_pose_(i, 0);
                    y2 = beacon_pose_(j, 1) - beacon_pose_(i, 1);

                    x2n = x2;/// std::pow(x2 * x2 + y2 * y2, 0.5);
                    y2n = y2;/// std::pow(x2 * x2 + y2 * y2, 0.5);

                    if (x2n > y2n) {
                        y3n = std::pow(1 / (1 + (y2n * y2n / x2n / x2n)), 0.5);
                        x3n = -y2n / x2n * y3n;

                        y3 = y3n * l1;
                        x3 = x3n * l1;

                        con_point_(index, 0) = x3 + offsetx;
                        con_point_(index, 1) = y3 + offsety;
                        ++index;

                        y3 = -y3;
                        x3 = -x3;
                        con_point_(index, 0) = x3 + offsetx;
                        con_point_(index, 1) = y3 + offsety;
                        ++index;


                    } else {
                        x3n = std::pow(1 / (1 + x2n * x2n / y2n / y2n), 0.5);
                        y3n = -x2n / y2n * x3n;

                        y3 = y3n * l1;
                        x3 = x3n * l1;

                        con_point_(index, 0) = x3 + offsetx;
                        con_point_(index, 1) = y3 + offsety;
                        ++index;

                        y3 = -y3;
                        x3 = -x3;
                        con_point_(index, 0) = x3 + offsetx;
                        con_point_(index, 1) = y3 + offsety;
                        ++index;

                    }
//                    if (isnan(con_point_(0, 1)) || isnan(con_point_(1, 1))) {
//                        std::cout << offsetx << std::endl;
//                        std::cout << offsetx << std::endl;
//
//                        std::cout << x2n << std::endl;
//                        std::cout << y2n << std::endl;
//
//                        std::cout << x2 << std::endl;
//                        std::cout << y2 << std::endl;
//
//                        std::cout << x3n << std::endl;
//                        std::cout << y3n << std::endl;
//
//                        std::cout << x3 << std::endl;
//                        std::cout << y3 << std::endl;
//
//                        std::cout << l1 << std::endl;
//
//                        std::cout << "------------------------" << std::endl;
//
//
//                    }

                }

            }
        }

        return true;
    }


    bool OwnParticleFilter::InitialState(Eigen::Vector2d real_pose) {
//        std::cout << "real pose: " << real_pose.transpose() << std::endl;
        state_(0) = real_pose(0);
        state_(1) = real_pose(1);

        for (int i(0); i < beacon_pose_.rows(); ++i) {
            state_(i + 2) = (Two2Three(real_pose) - beacon_pose_.block(i, 0, 1, 3)).norm();
        }

        for (int j(0); j < history_len_; ++j) {
            state_history_.push_back(state_);
        }

        particle_mx_.resize(particle_num_, state_.rows());
        weight_vec_.resize(particle_num_);

        std::vector<std::normal_distribution<>> normal_dis_vec;
//        std::vector<std::uniform_real_distribution<>> normal_dis_vec;
        for (int i(0); i < sigma_.rows(); ++i) {
            normal_dis_vec.push_back(std::normal_distribution<>(0.0, sigma_(i)));
//            normal_dis_vec.push_back(std::uniform_real_distribution<>(-sigma_(i) * 2, sigma_(i) * 2));
        }


        for (int i(0); i < weight_vec_.rows(); ++i) {
            weight_vec_(i) = 1.0;

            particle_mx_.block(i, 0, 1, particle_mx_.cols()) = state_.transpose();
//            std::cout << "random value:" << normal_dis_vec.at(2)(e_);
            for (int j(0); j < particle_mx_.cols(); ++j) {
                particle_mx_(i, j) += normal_dis_vec.at(j)(e_);
            }

        }
//        std::cout << particle_mx_<< std::endl;


        return true;

    }

    bool OwnParticleFilter::Sample() {

        std::vector<std::normal_distribution<>> normal_dis_vec;
        for (int i(0); i < sigma_.rows(); ++i) {
            normal_dis_vec.push_back(std::normal_distribution<>(0.0, sigma_(i)));

        }

        for (int i(0); i < weight_vec_.rows(); ++i) {
            for (int j(0); j < particle_mx_.cols(); ++j) {
                particle_mx_(i, j) += normal_dis_vec.at(j)(e_);
            }

        }
    }


    bool OwnParticleFilter::Evaluate(Eigen::VectorXd range_vec) {

        ComputeCPoint(range_vec);

        std::cout << "common point:" << con_point_ << std::endl;

        weight_vec_ /= weight_vec_.sum();

        state_.block(0, 2, 1, range_vec.rows()) = range_vec.transpose();
        Eigen::VectorXd Score(weight_vec_);
        for (int j(0); j < Score.rows(); ++j) {
            Score(j) = Likelihood(particle_mx_.block(j, 0, 1, particle_mx_.cols()), range_vec);
        }

        std::cout << "min x,y:" << particle_mx_.block(0, 0, particle_mx_.rows(), 1).minCoeff() <<
                  "," << particle_mx_.block(0, 1, particle_mx_.rows(), 1).minCoeff() << std::endl;
        std::cout << "max x,y:" << particle_mx_.block(0, 0, particle_mx_.rows(), 1).maxCoeff() <<
                  "," << particle_mx_.block(0, 1, particle_mx_.rows(), 1).maxCoeff() << std::endl;
        std::cout << "min score: " << Score.minCoeff() << "max score: " << Score.maxCoeff() << std::endl;
//        Score /= Score.sum();
        for (int i(0); i < weight_vec_.rows(); ++i) {
            weight_vec_(i) = weight_vec_(i) * Score(i);
        }
        std::cout << "min weight : " << weight_vec_.minCoeff() << "max weight:" << weight_vec_.maxCoeff() << std::endl;

        return true;
    }

    double OwnParticleFilter::Likelihood(Eigen::VectorXd guess_state, Eigen::VectorXd range_vec) {
        /*
         * Methond 1
         */
//        return 0.1;
        /*
         * Methond 2
         */
//        Eigen::Vector3d dist;
//        for (int i(0); i < 3; ++i) {
//            dist(i) = (Two2Three(Eigen::Vector2d(guess_state(0),guess_state(1))) - beacon_pose_.block(i, 0, 1, 3)).norm()-range_vec(i);
//            dist(i) /= (range_vec(i)+ 0.0000001);
//        }
//        double porbility = 1000000/((dist).norm());
////        std::cout << "P(): " << porbility << std::endl;
//        return porbility;
        /*
         * Methond 3
         */

//        Eigen::Vector3d dis, dis_err;
//        for (int i(0); i < 3; ++i) {
//
//            dis(i) = 0.0;
//            dis(i) += std::pow(guess_state(0) - beacon_pose_(i, 0), 2.0);
//            dis(i) += std::pow(guess_state(1) - beacon_pose_(i, 1), 2.0);
//            dis(i) += std::pow(z_offset_ - beacon_pose_(i, 2), 2.0);
//            dis(i) = std::pow(dis(i), 0.5);
//            dis_err(i) = std::abs(dis(i) - range_vec(i)) / (range_vec(i) + 0.0000000001);
//        }
////        double the_sum = dis_err.sum();
////        for(int i(0);i<3;++i)
////        {
////            if(the_sum < dis_err(i)*2)
////            {
////                dis_err(i) = (the_sum - dis_err(i))/2;
////                break;
////            }
////        }
//
//        return std::pow(1 / dis_err.norm(), 4.0);
        /*
         * Methond 4
         */
        double score(0.0);
        for (int i(0); i < con_point_.rows(); ++i) {

            score += 1 / 6.0 *
                     TwoDnormal(guess_state(0), guess_state(1), con_point_(i, 0), con_point_(i, 1), 0.0,
                                sigma_(0) * 2.0,
                                sigma_(1) * 2.0);
        }
//        if(isnan(score))
//        {
//            std::cout << "ERROR" << std::endl;
//            std::cout << "guess_state:"<<guess_state<<std::endl;
//            std::cout << "con_point:" << con_point_ << std::endl;
//        }
        return score;

    }

    Eigen::VectorXd OwnParticleFilter::GetResult() {

        Eigen::VectorXd tmp_state(state_);
        tmp_state *= 0.0;
        weight_vec_ /= weight_vec_.sum();

        for (int i(0); i < particle_mx_.rows(); ++i) {
            for (int j(0); j < particle_mx_.cols(); ++j) {
                tmp_state(j) += particle_mx_(i, j) * weight_vec_(i);
            }
        }
//        std::cout << tmp_state.transpose() << std::endl;
//        std::cout << particle_mx_.colwise().sum() << std::endl;
//        std::cout << tmp_state.transpose() << std::endl;
//        std::cout << state_.transpose() << std::endl;
//        std::cout << state_history_[2].transpose() << std::endl;
//        state_ = tmp_state;
//        std::cout << state_.transpose() << std::endl;
//        state_history_.pop_front();
//        state_history_.push_back(state_);

//        int index(0);
//        weight_vec_.maxCoeff(&index);
//
//        return particle_mx_.block(index,0,1,particle_mx_.cols());

        std::cout << "Neff:" << 1 / weight_vec_.norm() << std::endl;
        std::cout.precision(20);
        std::cout << "score:" << Likelihood(tmp_state, state_.block(1, 2, 1, 3)) << std::endl;

        return tmp_state;
    }

    bool OwnParticleFilter::ReSample() {


        weight_vec_ /= weight_vec_.sum();
        Eigen::MatrixXd tmp_matrix(particle_mx_);
        Eigen::VectorXd tmp_weight(weight_vec_);

        Eigen::VectorXd Beta(weight_vec_);

        for (int i(1); i < Beta.rows(); ++i) {
//            if(isnan(weight_vec_(i)))
//            {
//                std::cout << "weight:" << weight_vec_ << std::endl;
//            }
            Beta(i) = Beta(i - 1) + weight_vec_(i);
        }
        if (Beta.maxCoeff() < 1.0) {
            Beta(Beta.rows() - 1) = 1.0;
//            std::cout << Beta << std::endl;
        }

        std::uniform_real_distribution<double> u(0, 1);
        double tmp_rnd(0.0);
        for (int i(0); i < tmp_matrix.rows(); ++i) {
            tmp_rnd = u(e_);
            for (int j(0); j < Beta.rows(); ++j) {
                if (tmp_rnd < Beta(j)) {
//                    std::cout << "j: " << j << std::endl;
                    particle_mx_.block(i, 0, 1, particle_mx_.cols()) = tmp_matrix.block(j, 0, 1, tmp_matrix.cols());
                    weight_vec_(i) = tmp_weight(j);
                    break;
                }
                if (j == Beta.rows() - 1) {

                    //std::cout <<"rnd:" << tmp_rnd << "  beta:"<<Beta(j)<< std::endl;
                    weight_vec_.setOnes();
                    particle_mx_ = tmp_matrix;
//                    MYERROR("Unexpected run fork.")
                }
            }
        }

        return true;
    }

}





