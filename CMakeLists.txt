cmake_minimum_required(VERSION 2.8.3)
project(ro-location)



set(CMAKE_BUILD_TYPE  Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lpython2.7 -fopenmp -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  rospy
#  sensor_msgs
#  std_msgs
#)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake-2.8/Modules")#Add Eigen3 find cmake file here.

#openmp support
FIND_PACKAGE(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

#find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
find_package(MRPT REQUIRED gui obs maps
        nav opengl slam vision base )

find_package(Eigen3 REQUIRED)

find_package(PythonLibs 2.7 REQUIRED)


#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ro-location
##  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
        #        ${OpenCV_INCLUDE_DIRS}
        #        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        /usr/local/include
        /usr/include/
        #                /home/steve/Cpp_Extent/Cpp_Extent
        ${PYTHON_INCLUDE_PATH}
        ./include

)

link_directories(
        /usr/local/lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu/


)


add_executable(ro-location src/ro-localization_main.cpp)

#target_link_libraries(ro-location ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${MRPT_LIBS})
target_link_libraries(ro-location ${MRPT_LIBS})

add_executable(pf_test src/pf_test.cpp include/pf_test/EKF.h)

#target_link_libraries(pf_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${PYTHON_LIBRARIES})
target_link_libraries(pf_test ${PYTHON_LIBRARIES})

add_executable(methon_test src/methon_test.cpp include/pf_test/EKF.h)

#target_link_libraries(methon_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${PYTHON_LIBRARIES})
target_link_libraries(methon_test ${PYTHON_LIBRARIES})
