//Create by steve in 16-10-13 at 上午9:48
//
// Created by steve on 16-10-13.
//

#include "../include/Cpp_Extent/MyError.h"
#include "../include/Cpp_Extent/CSVReader.h"


#include "../include/Cpp_Extent/matplotlib_interface.h"

#include <omp.h>


namespace plt = matplotlibcpp;

/*
 * plt::plot(x,y,"r--")
 * (x,y should be a vector).
 *
 */

int main() {
    CSVReader gt("gt.csv"), beacon_set("beacon_set.csv"), uwb_range("uwb_range.csv");

    std::cout << gt.GetMatrix().GetRows() << " " << beacon_set.GetMatrix().GetRows() << " "
              << uwb_range.GetMatrix().GetRows() << std::endl;


    return 0;
}
