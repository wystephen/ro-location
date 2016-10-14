//Create by steve in 16-10-13 at 上午9:48
//
// Created by steve on 16-10-13.
//

#include "../include/Cpp_Extent/MyError.h"
#include "../include/Cpp_Extent/CSVReader.h"


#include "../include/Cpp_Extent/matplotlib_interface.h"

#include "../include/pf_test/OwnParticleFilter.hpp"

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





    for (int i(0); i < gtm.GetRows(); ++i) {
        gt_x.push_back(double(*gtm(i, 0)));
        gt_y.push_back(double(*gtm(i, 1)));


    }
    for (int i(0); i < range.GetRows(); ++i) {
        uwb_range_vec.push_back(Eigen::Vector3d(*range(i, 0), *range(i, 1), *range(i, 2)));
    }

    OPF::OwnParticleFilter opf(18000, apose, 1.12, 10);
    opf.InitialState(Eigen::Vector2d(gt_x[0], gt_y[0]));


    std::cout << "iwb size:" << uwb_range_vec.size() << "gt size :" << gt_x.size() << std::endl;

    double average(0.0);

    double last_dx(0.0),last_dy(0.0);

    for (int i(0); i < uwb_range_vec.size(); ++i) {
//        opf.Sample(last_dx,last_dy);
        opf.Sample();

        //Test if use a real range.
//        Eigen::Vector3d real_range(0, 0, 0);
//        Eigen::Vector3d rp(gt_x[i], gt_y[i], 1.12);
//        for (int k(0); k < 3; ++k) {
//            real_range(k) = (rp - apose.block(k, 0, 1, 3)).norm();
//        }
//
//        opf.Evaluate(Eigen::VectorXd(real_range));
//        std::cout << "Use Real Range:" << (real_range - uwb_range_vec[i]).norm() << std::endl;

        std::cout << "real pose:" << gt_x[i] << " " << gt_y[i] << std::endl;
        opf.Evaluate(Eigen::VectorXd(uwb_range_vec[i]));
        std::cout << "score of real_pose:" << opf.Likelihood(Eigen::VectorXd(Eigen::Vector3d(gt_x[i], gt_y[i], 1.0)),
                                                             Eigen::VectorXd(uwb_range_vec[i])) << std::endl;

        /*
         * Generate a Probobility Map
         */
        std::ofstream tmp_log("p_map_"+std::to_string(100 + i)+".txt");

        for(double x(-2.0);x<15.0;x+=0.05)
        {
            for(double y(-1.0);y<8.0;y+=0.05)
            {
                tmp_log<< opf.Likelihood(Eigen::VectorXd(Eigen::Vector3d(x, y, 1.0)),
                                         Eigen::VectorXd(uwb_range_vec[i]))<< " ";
            }
            tmp_log << std::endl;
        }
        tmp_log.close();


        Eigen::VectorXd p(opf.GetResult());

        f_x.push_back(p(0));
        f_y.push_back(p(1));

        if(i>3)
        {
            last_dx = f_x[i]-f_x[i-1];
            last_dy = f_y[i]-f_x[i-1];
        }


        err.push_back(std::pow((std::pow(gt_x[i] - f_x[i], 2) + std::pow(gt_y[i] - f_y[i], 2.0)), 0.5));

        average += err[i] / uwb_range_vec.size();

        opf.ReSample();

    }

    std::cout << f_x[0] << "                " << f_y[0] << std::endl;

    std::cout << " average err:" << average << std::endl;

    plt::subplot(2, 1, 1);
    plt::named_plot("a", f_x, f_y, "r+-");
    plt::named_plot("b", gt_x, gt_y, "g-");
    plt::subplot(2, 1, 2);
    plt::plot(err);
    plt::show();



    return 0;
}
