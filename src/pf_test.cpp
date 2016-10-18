//Create by steve in 16-10-13 at 上午9:48
//
// Created by steve on 16-10-13.
//

#include "../include/Cpp_Extent/MyError.h"
#include "../include/Cpp_Extent/CSVReader.h"


#include "Cpp_Extent/matplotlib_interface.h"

#include "pf_test/OwnParticleFilter.hpp"

#include <omp.h>

#include <vector>

#include <Eigen/Dense>

namespace plt = matplotlibcpp;

/*
 * plt::plot(x,y,"r--")
 * (x,y should be a vector).
 *
 */

int main() {
    bool is_view_likelihood(false);
    bool is_view_particle(false);
    CSVReader gt("gt.csv"), beacon_set("beacon_set.csv"), uwb_range("uwb_range.csv");

//    std::cout << gt.GetMatrix().GetRows() << " " << beacon_set.GetMatrix().GetRows() << " "
//              << uwb_range.GetMatrix().GetRows() << std::endl;

    auto gtm(gt.GetMatrix()), set(beacon_set.GetMatrix()), range(uwb_range.GetMatrix());
    std::vector<double> gt_x, gt_y;
    std::vector<Eigen::Vector3d> uwb_range_vec;
    std::vector<double> err, f_x, f_y;

    Eigen::MatrixXd apose;
    apose.resize(3, 3);

    for (int i(0); i < set.GetRows(); ++i) {
        for (int j(0); j < set.GetCols(); ++j) {
            apose(i, j) = *set(i, j);
        }
    }

    std::cout << "apose:" << std::endl << apose << std::endl;

    std::cout.precision(20);

//


    for (int i(0); i < gtm.GetRows(); ++i) {
        gt_x.push_back(double(*gtm(i, 0)));
        gt_y.push_back(double(*gtm(i, 1)));


    }
    for (int i(0); i < range.GetRows(); ++i) {
        uwb_range_vec.push_back(Eigen::Vector3d(*range(i, 0), *range(i, 1), *range(i, 2)));
    }

    OPF::OwnParticleFilter opf(30000, apose, 1.12, 10);
    opf.InitialState(Eigen::Vector2d(gt_x[0], gt_y[0]));


    std::cout << "iwb size:" << uwb_range_vec.size() << "gt size :" << gt_x.size() << std::endl;

    double average(0.0);

    double last_dx(0.0), last_dy(0.0);

    std::vector<double> real_score, result_score;

    for (int i(0); i < uwb_range_vec.size(); ++i) {
//        opf.Sample(last_dx,last_dy);

//        opf.Evaluate(Eigen::VectorXd(uwb_range_vec[i]));

        /*
         * SAMPLE
         */
        if (i < 4)
            opf.Sample();
        else {
            opf.Sample((gt_x[i] - gt_x[i - 4]) / 4.0, (gt_y[i] - gt_y[i - 4]) / 4.0);
        }
//        if(i<5)
//        {
//            opf.Sample();
//        }else{
//            opf.Sample((f_x[i-1]-f_x[i-4])/3.0,(f_y[i-1]-f_y[i-4])/3.0);
//        }



        /*
         * Test if use a real range.
         */
//        Eigen::Vector3d real_range(0, 0, 0);
//        Eigen::Vector3d rp(gt_x[i], gt_y[i], 1.12);
//        for (int k(0); k < 3; ++k) {
//            real_range(k) = (rp - apose.block(k, 0, 1, 3)).norm();
//        }
//
//        opf.Evaluate(Eigen::VectorXd(real_range));
//        std::cout << "Use Real Range:" << (real_range - uwb_range_vec[i]).norm() << std::endl;

        /*
         * EVALUATE
         */
        opf.Evaluate(Eigen::VectorXd(uwb_range_vec[i]));
        /*
         * End EVALUATE
         */


        /*
         * Output particle image
         */
        if (i % 3 == 0 && is_view_particle) {
            std::vector<std::vector<double>> bx, by;
            bx.resize(3);
            by.resize(3);

            for (double theta(-M_PI); theta < M_PI; theta += 0.05) {
                for (int j(0); j < 3; ++j) {
                    double dx, dy;
                    dx = std::sin(theta) * uwb_range_vec[i](j);
                    dy = std::cos(theta) * uwb_range_vec[i](j);
                    bx[j].push_back(dx + apose(j, 0));
                    by[j].push_back(dy + apose(j, 1));
                }
            }

            for (int j(0); j < 3; ++j) {
                plt::plot(bx[j], by[j], "y-");
            }

            std::vector<double> tx, ty;

            double mx(0.0), my(0.0);
            double max_score(-100.0);
            for (double y(16.0); y > -3; y -= 0.05) {
                for (double x(-4.0); x < 20.0; x += 0.05) {
                    double the_sco = opf.Likelihood(Eigen::VectorXd(Eigen::Vector3d(x, y, 1.0)),
                                                    Eigen::VectorXd(uwb_range_vec[i]));
                    if (the_sco > max_score) {
                        max_score = the_sco;
                        mx = x;
                        my = y;
                    }
                }
            }


            tx.push_back(mx);
            ty.push_back(my);

            if (i > 0) {
                double dx(gt_x[i] - gt_x[i - 1]), dy(gt_y[i] - gt_y[i - 1]);

                plt::title("distance is:" + std::to_string(std::pow(dx * dx + dy * dy, 0.5))
                           + " i is:" + std::to_string(i));
            }

            plt::plot(gt_x, gt_y, "g-");
            opf.SaveParicleAsImg(gt_x[i], gt_y[i]);

            plt::plot(tx, ty, "k*");
            plt::show();
        }

        /*
        * END Output particle image
        */
//        plt::save("test"+std::to_string(i)+"123.jpg");



        std::cout << "real pose:" << gt_x[i] << " " << gt_y[i] << std::endl;
        real_score.push_back(opf.Likelihood(Eigen::VectorXd(Eigen::Vector3d(gt_x[i], gt_y[i], 1.0)),
                                            Eigen::VectorXd(uwb_range_vec[i])));

        std::cout << "score of real_pose:" << real_score[i] << std::endl;
        /*
         * Generate a Probobility Map
         */
        if (is_view_likelihood) {
            std::ofstream tmp_log("../tmpdata/p_map_" + std::to_string(100 + i) + ".txt");

            for (double y(16.0); y > -3; y -= 0.05) {
                for (double x(-4.0); x < 20.0; x += 0.05) {
                    tmp_log << opf.Likelihood(Eigen::VectorXd(Eigen::Vector3d(x, y, 1.0)),
                                              Eigen::VectorXd(uwb_range_vec[i])) << " ";
                }
                tmp_log << std::endl;
            }
            tmp_log.close();
        }



        Eigen::VectorXd p(opf.GetResult());

        result_score.push_back(opf.Likelihood(Eigen::VectorXd(Eigen::Vector3d(p(0), p(1), 1.12)),
                                              Eigen::VectorXd(uwb_range_vec[i])));

        std::cout << "Result score:" << result_score[i] << std::endl;

        f_x.push_back(p(0));
        f_y.push_back(p(1));

        if (i > 3) {
            last_dx = f_x[i] - f_x[i - 1];
            last_dy = f_y[i] - f_x[i - 1];
        }


        err.push_back(std::pow((std::pow(gt_x[i] - f_x[i], 2) + std::pow(gt_y[i] - f_y[i], 2.0)), 0.5));


        if (!isnan(err[i]))
            average += err[i] / uwb_range_vec.size();

        /*
         * ReSample
         * Needn't resample in every time steps.
         */
//        opf.ReSample();


    }

    std::cout << f_x[0] << "                " << f_y[0] << std::endl;

    std::cout << " average err:" << average << std::endl;

    plt::subplot(2, 2, 1);
    plt::named_plot("a", f_x, f_y, "r+-");
    plt::named_plot("b", gt_x, gt_y, "g-");
    plt::grid(true);

    plt::subplot(2, 2, 2);
    plt::plot(err, "r+-");
    plt::grid(true);
    plt::title("average err:" + std::to_string(average));

    plt::subplot(2, 2, 3);
    plt::grid(true);
    plt::plot(real_score, "g+-");
    plt::plot(result_score, "r+-");
    plt::title("score : red:result  green:real");


    plt::show();


    return 0;
}
