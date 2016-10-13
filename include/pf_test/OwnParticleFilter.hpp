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
                sigma_(i) = 0.2;
            }


        }

        bool InitialState(Eigen::Vector2d real_pose);

        bool Sample();

        bool Evaluate(Eigen::VectorXd range_vec);

        double Likelihood(Eigen::VectorXd guess_state, Eigen::VectorXd range_vec);

        Eigen::VectorXd GetResult();

        bool ReSample();


    protected:

        long particle_num_ = 1000;//particle number.

//        std::deque<Eigen::VectorXd> range_history_;//save a sequence of range.

        std::deque<Eigen::VectorXd> state_history_;//save a sequence of state.

        Eigen::VectorXd state_;//current state vec

        Eigen::MatrixXd beacon_pose_;//every row represts a beacon's position.

        double z_offset_;//offset of tag at axis z.

        long history_len_ = 10;//the size of state_history deque.

        Eigen::MatrixXd particle_mx_;//The matrix saved the particle

        Eigen::VectorXd weight_vec_;//The vector saved the particle

        std::default_random_engine e_;


        Eigen::VectorXd sigma_;//sigma of state_



    private:
        /*
         *
         */
        Eigen::Vector3d Two2Three(Eigen::Vector2d pose) {
            return Eigen::Vector3d(pose(0), pose(1), z_offset_);
        }


    };


    bool OwnParticleFilter::InitialState(Eigen::Vector2d real_pose) {
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
        for (int i(0); i < sigma_.rows(); ++i) {
            normal_dis_vec.push_back(std::normal_distribution<>(0.0, sigma_(i)));
        }


        for (int i(0); i < weight_vec_.rows(); ++i) {
            weight_vec_(i) = 1.0;

            particle_mx_.block(i, 0, 1, particle_mx_.cols()) = state_.transpose();

            for (int j(0); j < particle_mx_.cols(); ++j) {
                particle_mx_(i, j) = state_(j) + normal_dis_vec.at(j)(e_);
            }

        }
        std::cout << particle_mx_.size() << std::endl;


        return true;

    }

    bool OwnParticleFilter::Sample() {
        std::vector<std::normal_distribution<>> normal_dis_vec;
        for (int i(0); i < sigma_.rows(); ++i) {
            normal_dis_vec.push_back(std::normal_distribution<>(0.0, sigma_(i)));
        }

        for (int i(0); i < weight_vec_.rows(); ++i) {
            weight_vec_(i) = 1.0;
            particle_mx_.block(i, 0, 1, particle_mx_.cols()) = state_;
            for (int j(0); j < particle_mx_.cols(); ++j) {
                particle_mx_(i, j) += normal_dis_vec.at(j)(e_);
            }

        }
    }

    bool OwnParticleFilter::Evaluate(Eigen::VectorXd range_vec) {
//        weight_vec_ /= weight_vec_.sum();

        for (int i(0); i < weight_vec_.rows(); ++i) {
            weight_vec_(i) *= Likelihood(particle_mx_.block(i, 0, 1, particle_mx_.cols()), range_vec);
        }

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
        Eigen::Vector3d dist;
        for (int i(0); i < 3; ++i) {
            dist(i) = (Two2Three(guess_state.block(0, 0, 1, 2)) - beacon_pose_.block(i, 0, 1, 3)).norm();
        }
        return 1 / (dist - range_vec).norm();

    }

    Eigen::VectorXd OwnParticleFilter::GetResult() {
        weight_vec_ /= weight_vec_.sum();
//        particle_mx_.rowwise() *= weight_vec_;
        Eigen::VectorXd tmp_state(state_);
        tmp_state.setZero();

        for (int i(0); i < particle_mx_.rows(); ++i) {
            tmp_state += particle_mx_.block(i, 0, 1, particle_mx_.cols()) * weight_vec_(i);
        }

        state_history_.push_back(tmp_state);
        state_history_.pop_front();


        return tmp_state;
    }

    bool OwnParticleFilter::ReSample() {

        Eigen::MatrixXd tmp_matrix(particle_mx_);
        Eigen::VectorXd tmp_weight(weight_vec_);
        weight_vec_ /= weight_vec_.sum();

        Eigen::VectorXd Beta(weight_vec_);

        for (int i(1); i < Beta.rows(); ++i) {
            Beta(i) = Beta(i - 1) + weight_vec_(i);
        }

        std::uniform_real_distribution<double> u(0, 1);
        for (int i(0); i < tmp_matrix.rows(); ++i) {
            double tmp_rnd = u(e_);
            for (int j(0); j < Beta.rows(); ++j) {
                if (tmp_rnd < Beta(j)) {
                    particle_mx_.block(i, 0, 1, particle_mx_.cols()) = tmp_matrix.block(j, 0, 1, tmp_matrix.cols());
                    weight_vec_(i) = tmp_weight(j);
                    break;
                }
            }

        }


        return true;
    }

}





