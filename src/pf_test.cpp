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

    for (int i(0); i < gtm.GetRows(); ++i) {
        gt_x.push_back(double(*gtm(i, 0)));
        gt_y.push_back(double(*gtm(i, 1)));
//        std::cout << gt_x.size() << "   " << gt_y.size() << std::endl;
//        std::cout << "gt_x,gt_y: " << gt_x.at(i) << " " << gt_y.at(i) << std::endl;
//

    }
    for (int i(0); i < range.GetRows(); ++i) {
        uwb_range_vec.push_back(Eigen::Vector3d(*range(i, 0), *range(i, 1), *range(i, 2)));
    }

    OPF::OwnParticleFilter opf(1000, apose, 1.12, 10);
    opf.InitialState(Eigen::Vector2d(gt_x[0], gt_y[0]));

    for (int i(0); i < uwb_range_vec.size(); ++i) {
        opf.Sample();

        opf.Evaluate(Eigen::VectorXd(uwb_range_vec[i]));

        Eigen::VectorXd p(opf.GetResult());

        f_x.push_back(p(0));
        f_y.push_back(p(1));

        err.push_back(std::pow((std::pow(gt_x[i] - f_x[i], 2) + std::pow(gt_y[i] - f_y[i], 2.0)), 0.5));

        opf.ReSample();


    }

    std::cout << " eee" << std::endl;

    plt::subplot(2, 1, 1);
    plt::named_plot("a", f_x, f_y, "r--");
    plt::named_plot("b", gt_x, gt_y, "g-");
    plt::subplot(2, 1, 2);
    plt::plot(err);
    plt::show();



    return 0;
}
