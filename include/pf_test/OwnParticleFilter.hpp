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

#include "../Cpp_Extent/matplotlib_interface.h"

namespace plt = matplotlibcpp;


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
                sigma_(i) = 0.4;
            }


        }

        bool InitialState(Eigen::Vector2d real_pose);

        bool Sample();

        bool Sample(double dx,double dy);

        bool EnhanceSample(Eigen::VectorXd range);

        bool Evaluate(Eigen::VectorXd range_vec);

        double Likelihood(Eigen::VectorXd guess_state, Eigen::VectorXd range_vec);

        Eigen::VectorXd GetResult();

        bool ReSample();

        bool ComputeCPoint(Eigen::VectorXd range);

        bool SaveParicleAsImg(double x, double y);

        int imge_index_ = 0;

        double last_best_x_, last_best_y_;

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

        std::default_random_engine e_;//Random generator engine.

        Eigen::VectorXd sigma_;//sigma of state_

        Eigen::Matrix<double, 6, 2> con_point_;

        Eigen::VectorXd avg_range_;

    private:
        /*
         *
         */
        Eigen::Vector3d Two2Three(Eigen::Vector2d pose) {

            return Eigen::Vector3d(pose(0), pose(1), z_offset_);
        }

        double TwoDnormal(double x, double y, double miu1, double miu2, double rho, double sigma1, double sigma2);

        double NormalPDF(double x, double miu, double sigma) {
            return 1 / std::sqrt(2.0 * M_PI) / sigma * std::exp(-std::pow(x - miu, 2.0) / 2 / sigma / sigma);
        }

        double LogNormalPDF(double x, double miu, double sigma) {
            return std::log(1 / std::sqrt(2.0 * M_PI) / sigma) - (std::pow(x - miu, 2.0) / 2 / sigma / sigma);
        }


        /*
         * Some error.
         */
        double GEVpdf() {
            return 0.0;
        }


    };

    double OwnParticleFilter::TwoDnormal(double x,
                                         double y,
                                         double miu1,
                                         double miu2,
                                         double rho,
                                         double sigma1,
                                         double sigma2) {

        if (miu1 > 1000) {
            return 0.0000000000000001;
        }


        double para1, para2;
        para1 = 2.0 * M_PI * sigma1 * sigma2 * std::pow(1.0 - rho * rho, 0.5);


        para2 = -1.0 / 2.0 / (1.0 - rho * rho) *
                (std::pow(x - miu1, 2.0) / sigma1 / sigma1 + 2.0 * rho * (x - miu1) * (y - miu2) / sigma1 / sigma2
                 + std::pow(y - miu2, 2.0) / sigma2 / sigma2);


        return std::log(1 / para1) * (para2);
    }

    /*
     * Compute common points of each pairs of range.
     */
    bool OwnParticleFilter::ComputeCPoint(Eigen::VectorXd range) {
        /*
         * Step 1: convert range in 3d to range_2d.
         */
        Eigen::VectorXd range2d(range);

        for (int i(0); i < range.size(); ++i) {
            range2d(i) = std::pow((range(i) * range(i) - std::pow(beacon_pose_(i, 2) - z_offset_, 2)), 0.5);
            if (isnan(range2d(i))) range2d(i) = 0.0;//When range(i) is
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
                if (range2d(j) + range2d(i) < dis_mat(i, j)) {
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

        last_best_x_ = real_pose(0);
        last_best_y_ = real_pose(1);

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

//        std::vector<std::normal_distribution<>> normal_dis_vec;
//        for (int i(0); i < sigma_.rows(); ++i) {
//            normal_dis_vec.push_back(std::normal_distribution<>(0.0, sigma_(i)));
//
//        }
//
//        std::normal_distribution<> rnd_get(0.0, sigma_(0));
//        /*
//         * Sample Methon 1.
//         */
//
//        for (int i(0); i < weight_vec_.rows(); ++i) {
//            double dis_tance(0.0);
//            double rnd_val(0.0);
//            for (int j(0); j < particle_mx_.cols(); ++j) {
//                rnd_val = rnd_get(e_);
////                std::cout << "rnd_val:" << rnd_val << std::endl;
//                particle_mx_(i, j) += rnd_val;
////                if (j == 0 or j == 1)
////                    dis_tance += rnd_val * rnd_val;
//            }
//        }

////            weight_vec_(i) *= 1/std::sqrt(2.0 * M_PI) / 0.1 * std::exp(-std::pow(dis_tance-0.15,2.0)/2/0.1/0.1);
//
//        }
        /*
         * Sample Methon 2
         */
//        for (int i(0); i < weight_vec_.rows(); ++i) {
//            double dis_tance(0.0);
//            double rnd_val(0.0);
//            for (int j(0); j < particle_mx_.cols(); ++j) {
//                rnd_val = rnd_get(e_);
//                particle_mx_(i,j) -= 0.5 * (particle_mx_(i,j) - state_(j));
//                particle_mx_(i, j) += rnd_val/5.0;
//
//            }
////            weight_vec_(i) *= 1/std::sqrt(2.0 * M_PI) / 0.1 * std::exp(-std::pow(dis_tance-0.15,2.0)/2/0.1/0.1);
//
//        }

        /*
         * Sample Methon 3
         */
//        std::normal_distribution<> nor_get(0.1, 0.03);
//        std::uniform_real_distribution<double> angle_get(-M_PI, M_PI);
//
//        for (int i(0); i < weight_vec_.rows(); ++i) {
//            double dis(nor_get(e_));
//            double angle(angle_get(e_));
//            double delta_x(dis * std::cos(angle));
//            double delta_y(dis * std::sin(angle));
////            std::cout << "de x:" << delta_x << " de y: " << delta_y << std::endl;
//            particle_mx_(i, 0) += delta_x;
//            particle_mx_(i, 1) += delta_y;
//        }
        /*
         * Sample Methon 4
         */
//        std::normal_distribution<> nor_get(0.0, 0.1615);
        std::normal_distribution<> nor_get(0.0, 0.9815);
        std::uniform_real_distribution<double> angle_get(0.0, M_PI);

        for (int i(0); i < weight_vec_.rows(); ++i) {
            double dis(nor_get(e_));
            double angle(angle_get(e_));
            double delta_x(dis * std::cos(angle));
            double delta_y(dis * std::sin(angle));
//            std::cout << "de x:" << delta_x << " de y: " << delta_y << std::endl;
            particle_mx_(i, 0) += delta_x;
            particle_mx_(i, 1) += delta_y;
            for (int j(2); j < particle_mx_.cols(); ++j) {

                auto b = [&] {
                    std::uniform_real_distribution<double> rnd(0, 1);
                    if (rnd(e_) < 0.05) {
                        return particle_mx_(i, j) + (rnd(e_) - 0.5) * 10.0;
                    } else {

                    }
                };
                b();
            }
        }
        /*
         * Sample Methon 5
         */
//        std::normal_distribution<> nor_get(0.1, 0.2);
//
//        for (int i(0); i < weight_vec_.rows(); ++i) {
//            double dis(nor_get(e_));
//            double angle(std::atan2(last_best_y_ - particle_mx_(i, 1), last_best_x_ - particle_mx_(i, 0)));
//            double ddx(last_best_x_ - particle_mx_(i, 0)), ddy(last_best_y_ - particle_mx_(i, 1));
//            double val = std::pow(ddx * ddx + ddy * ddy, 0.5);
////            ddx /=val;
////            ddy /=val;
//
//            double delta_x(dis * ddx);// + dis * std::cos(angle));
//            double delta_y(dis * ddy);//+ dis * std::sin(angle));
////            std::cout << "de x:" << delta_x << " de y: " << delta_y << std::endl;
//            particle_mx_(i, 0) += delta_x;
//            particle_mx_(i, 1) += delta_y;
//        }
        /*
         * Sample Methon6
         */
//        std::normal_distribution<> nor_get(0.0, 0.6);
//
//        for (int i(0); i < weight_vec_.rows(); ++i) {
//            particle_mx_(i, 0) = (last_best_x_+state_(0))/2.0 + nor_get(e_);
//            particle_mx_(i, 1) = (last_best_y_ + state_(1)) /2.0 + nor_get(e_);
//        }
        /*
         * Sample Methon7
         */

//        std::normal_distribution<> nor_get(0.0, 0.1815);
//        std::uniform_real_distribution<double> angle_get(0.0, M_PI);
//        std::uniform_real_distribution<double> unif_get(0.005, 1.1);
//        //con_point_//
//
//        std::uniform_int_distribution<int> index_get(0, 5);
//
//        for (int i(0); i < weight_vec_.rows(); ++i) {
//            int index = index_get(e_);
//            if (con_point_(index, 0) > 10000 ) {
//                double dis(nor_get(e_));
//                double angle(angle_get(e_));
//                double delta_x(dis * std::cos(angle));
//                double delta_y(dis * std::sin(angle));
//                particle_mx_(i, 0) += delta_x;
//                particle_mx_(i, 1) += delta_y;
//            } else {
//                double beta = unif_get(e_);
//
//                particle_mx_(i, 0) += beta * (con_point_(index, 0) - particle_mx_(i, 0));
//                particle_mx_(i, 1) += beta * (con_point_(index, 1) - particle_mx_(i, 1));
//            }
//        }

        return true;

    }

    bool OwnParticleFilter::EnhanceSample(Eigen::VectorXd range) {
        ComputeCPoint(range);
        std::normal_distribution<> nor_get(0.0, 0.1815);
        std::uniform_real_distribution<double> angle_get(0.0, M_PI);
        std::uniform_real_distribution<double> unif_get(0.005, 1.1);
        //con_point_//

        std::uniform_int_distribution<int> index_get(0, 5);

        for (int i(0); i < weight_vec_.rows(); ++i) {
            int index = index_get(e_);
            if (con_point_(index, 0) > 10000) {
                double dis(nor_get(e_));
                double angle(angle_get(e_));
                double delta_x(dis * std::cos(angle));
                double delta_y(dis * std::sin(angle));
                particle_mx_(i, 0) += delta_x;
                particle_mx_(i, 1) += delta_y;
            } else {
                double beta = unif_get(e_);

                particle_mx_(i, 0) += beta * (con_point_(index, 0) - particle_mx_(i, 0));
                particle_mx_(i, 1) += beta * (con_point_(index, 1) - particle_mx_(i, 1));
            }
        }

        return true;
    }

    bool OwnParticleFilter::Sample(double dx, double dy) {
        Eigen::Vector2d delta(dx,dy);
//        delta = delta.sum();
        /*
         * randon bias
         */

        std::vector<std::normal_distribution<>> normal_dis_vec;
        for (int i(0); i < sigma_.rows(); ++i) {
            if(i<2)
            {
                normal_dis_vec.push_back(std::normal_distribution<>(delta(i), sigma_(i)));

            }
            else{
                normal_dis_vec.push_back(std::normal_distribution<>(0.0, sigma_(i)));

            }

        }

        for (int i(0); i < weight_vec_.rows(); ++i) {
            for (int j(0); j < particle_mx_.cols(); ++j) {
                particle_mx_(i, j) += normal_dis_vec.at(j)(e_);
                auto b = [&] {
                    std::uniform_real_distribution<double> rnd(0, 1);
                    if (rnd(e_) < 0.05) {
                        return particle_mx_(i, j) + (rnd(e_) - 0.5) * 10.0;
                    } else {

                    }
                };
                if (j > 2) b();
            }

        }


        return true;
    }


    bool OwnParticleFilter::Evaluate(Eigen::VectorXd range_vec) {

        /*
         * Compute Common points.
         */
//        ComputeCPoint(range_vec);
//        std::cout << "common point:" << con_point_ << std::endl;

        if (avg_range_.size() < 1) {
            avg_range_ = range_vec;
        } else {
            avg_range_ *= 0.8;
            avg_range_ += 0.2 * range_vec;
        }

        weight_vec_ /= weight_vec_.sum();

//        for(int i(2);i<sigma_.rows();++i)
//        {
//            sigma_(i) = sigma_(i) * 0.5 + std::abs(state_(i)-range_vec(i-2))*0.5;
//        }

        state_.block(0, 2, 1, range_vec.rows()) = range_vec.transpose();
        Eigen::VectorXd Score(weight_vec_);
        for (int j(0); j < Score.rows(); ++j) {
            Score(j) = Likelihood(particle_mx_.block(j, 0, 1, particle_mx_.cols()), range_vec);
//            Score(j) = std::exp(Score(j)/10000.0);
        }
        Score /= Score.sum();

//        double s_mean(Score.mean());
//        for (int j(0); j < Score.rows(); ++j) {
//            if (Score(j) < s_mean) {
//                Score(j) = 0.0;
//            }
//        }


        int best_score_index(0.0);
        Score.maxCoeff(&best_score_index);

        last_best_x_ = particle_mx_(best_score_index, 0);
        last_best_y_ = particle_mx_(best_score_index, 1);

        std::cout << "SUM OF SCORE:" << Score.sum() << std::endl;

        std::cout << "min x,y:" << particle_mx_.block(0, 0, particle_mx_.rows(), 1).minCoeff() <<
                  "," << particle_mx_.block(0, 1, particle_mx_.rows(), 1).minCoeff() << std::endl;
        std::cout << "max x,y:" << particle_mx_.block(0, 0, particle_mx_.rows(), 1).maxCoeff() <<
                  "," << particle_mx_.block(0, 1, particle_mx_.rows(), 1).maxCoeff() << std::endl;
        std::cout << "min score: " << Score.minCoeff() << "max score: " << Score.maxCoeff() << std::endl;


        /*
         * Update weight.
         */
        std::cout << "min weight : " << weight_vec_.minCoeff() << "max weight:" << weight_vec_.maxCoeff() << std::endl;
        for (int i(0); i < weight_vec_.size(); ++i) {
            weight_vec_(i) *= (Score(i));
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
//        double score(0.0);
//        for (int i(0); i < con_point_.rows(); ++i) {
//
//            score += 1 / 6.0 *
//                     TwoDnormal(guess_state(0), guess_state(1), con_point_(i, 0), con_point_(i, 1), 0.0,
//                                sigma_(0) ,
//                                 sigma_(1) );
//        }
////        if(isnan(score))
////        {
////            std::cout << "ERROR" << std::endl;
////            std::cout << "guess_state:"<<guess_state<<std::endl;
////            std::cout << "con_point:" << con_point_ << std::endl;
////        }
//
////        score = std::pow(4.0,score);
//        return score;
        /*
         * Methond 5 in paper
         */
        std::normal_distribution<> n(0, 0.012);

        double ret(0.0);
        Eigen::Vector3d dis;
        for (int j(0); j < 3; ++j) {
            sigma_(j + 2) = 1.0;
        }
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

            ret += (NormalPDF(range_vec(i), dis(i) + f() /*+ guess_state(2 + i)/*+ n(e_)*/, 2.1) /*+ 1e-159*/);


        }
//        std::cout << "ret " << ret << std::endl;


//        return std::pow(2.0,ret);
        return (ret);


    }

    Eigen::VectorXd OwnParticleFilter::GetResult() {

        Eigen::VectorXd tmp_state(state_);
        tmp_state.setZero();
        weight_vec_ /= weight_vec_.sum();
        if (isnan(weight_vec_.sum())) {
            weight_vec_.setOnes();
            weight_vec_ /= weight_vec_.sum();
        }
        for (int i(0); i < particle_mx_.rows(); ++i) {
            for (int j(0); j < particle_mx_.cols(); ++j) {
                tmp_state(j) += particle_mx_(i, j) * weight_vec_(i);
            }
        }
        {
            //        std::cout << tmp_state.transpose() << std::endl;
            //        std::cout << particle_mx_.colwise().sum() << std::endl;
            //        std::cout << tmp_state.transpose() << std::endl;
            //        std::cout << state_.transpose() << std::endl;
            //        std::cout << state_history_[2].transpose() << std::endl;
            //        state_ = tmp_state;
            //        std::cout << state_.transpose() << std::endl;
//                    state_history_.pop_front();
//                    state_history_.push_back(state_);

            //        int index(0);
            //        weight_vec_.maxCoeff(&index);
            //
            //        return particle_mx_.block(index,0,1,particle_mx_.cols());
        }


        std::cout << "Neff:" << 1 / weight_vec_.norm() / weight_vec_.norm() << std::endl;

//        while (1 / weight_vec_.norm() / weight_vec_.norm() < particle_num_ / 100.0) {
//            ReSample();
//            std::cout << "Neff:" << 1 / weight_vec_.norm() / weight_vec_.norm() << std::endl;
//        }

//        int max_index(0);
//        weight_vec_.maxCoeff(&max_index);
//
//
//        return particle_mx_.block(max_index,0,1,5);
        state_(0) = tmp_state(0);
        state_(1) = tmp_state(1);
        return tmp_state;
    }

    bool OwnParticleFilter::ReSample() {


        /*
         * RESAMPLE METHON 1
         */
//        weight_vec_ /= weight_vec_.sum();
//        Eigen::MatrixXd tmp_matrix(particle_mx_);
//        Eigen::VectorXd tmp_weight(weight_vec_);
//
//        Eigen::VectorXd Beta(weight_vec_);
//
//        for (int i(1); i < Beta.rows(); ++i) {
//
//            Beta(i) = Beta(i - 1) + weight_vec_(i);
//        }
//        if (Beta.maxCoeff() < 1.0) {
//            Beta(Beta.rows() - 1) = 1.0;
////            std::cout << Beta << std::endl;
//        }
//
//        std::uniform_real_distribution<double> uuu(0, 1);
//        double tmp_rnd(0.0);
//
//        for (int i(0); i < particle_mx_.rows(); ++i) {
//            tmp_rnd = uuu(e_);
//            for (int j(0); j < Beta.rows(); ++j) {
//                if (tmp_rnd < Beta(j)) {
//                    for (int k(0); k < particle_mx_.cols(); ++k) {
//                        particle_mx_(i, k) = tmp_matrix(j, k);
//                    }
////                    particle_mx_.block(i, 0, 1, particle_mx_.cols()) = tmp_matrix.block(j, 0, 1, tmp_matrix.cols());
//                    weight_vec_(i) = tmp_weight(j);
//                    break;
//                }
//                if (j == Beta.rows() - 1) {
//
//                    std::cout << "rnd:" << tmp_rnd << "  beta:" << Beta(j) << std::endl;
////                    weight_vec_.setOnes();
////                    particle_mx_ = tmp_matrix;
//                    MYERROR("Unexpected run fork.")
//                    return true;
//                }
//            }
//        }
//
//        std::cout << "min weight after resample:" << weight_vec_.minCoeff() << std::endl;

//        weight_vec_.setOnes();
//        weight_vec_ /= weight_vec_.sum();

        /*
         * RESAMPLE METHON2
         *
         */

        weight_vec_ /= weight_vec_.sum();
        Eigen::MatrixXd tmp_matrix(particle_mx_);
        Eigen::VectorXd tmp_weight(weight_vec_);

        std::uniform_real_distribution<double> uuu(0, 1);
        double tmp_rnd(0.0);

        for (int i = 0; i < particle_mx_.rows(); ++i) {
            tmp_rnd = uuu(e_);
            int index = -1;

            do {
                index++;
                tmp_rnd -= tmp_weight(index);
            } while (tmp_rnd > 0.0);

            weight_vec_(i) = tmp_weight(index);
            for (int k(0); k < particle_mx_.cols(); ++k) {
                particle_mx_(i, k) = tmp_matrix(index, k);
            }

        }




        return true;
    }

    bool OwnParticleFilter::SaveParicleAsImg(double x, double y) {

        std::string path("particleimg/sa" + std::to_string(100 + imge_index_) + "save.jpg");
        imge_index_++;

        std::vector<double> lx, ly, gx, gy;
        gx.push_back(x);
        gy.push_back(y);

        Eigen::MatrixXd tmp_matrix(particle_mx_);
        Eigen::VectorXd tmp_weight(weight_vec_);

        tmp_weight /= tmp_weight.sum();

        Eigen::VectorXd Beta(weight_vec_);


        for (int i(1); i < Beta.rows(); ++i) {
            Beta(i) = Beta(i - 1) + tmp_weight(i);
        }
        if (Beta.maxCoeff() < 1.0) {
            Beta(Beta.rows() - 1) = 1.0;
//            std::cout << Beta << std::endl;
        }

        std::uniform_real_distribution<double> uuu(0.0, 1.0);
        double tmp_rnd(0.0);
        for (int i(0); i < 2100; ++i) {
            tmp_rnd = uuu(e_);
            for (int j(0); j < Beta.rows(); ++j) {
                if (tmp_rnd < Beta(j)) {

                    lx.push_back(tmp_matrix(j, 0));
                    ly.push_back(tmp_matrix(j, 1));

                    break;
                }
                if (j == Beta.rows() - 1) {
                    MYERROR("Unexpected run fork.")
//                    return true;

                }
            }

        }


        std::cout << lx.size() << "     ------     " << ly.size() << std::endl;



        plt::named_plot("particle" + std::to_string(imge_index_), lx, ly, "+r");
//        plt::plot(lx,ly,"+r");

        plt::named_plot("true" + std::to_string(imge_index_), gx, gy, "b*");

        plt::xlim(-3.0, 15.0);
        plt::ylim(-3.0, 15.0);

        plt::grid(true);
//        MYCHECK(1);

//        plt::show();
        plt::legend();
//
//        plt::save(path);





        return true;

    }

}
