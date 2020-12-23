CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(g2o_with_line_opt)
SET(LIB_NAME g2o_line_merge)
SET(g2o_C_FLAGS)
SET(g2o_CXX_FLAGS)

SET(CMAKE_BUILD_TYPE Release)
# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

SET (G2O_LIB_TYPE STATIC)


#SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -pthread -O3 -march=native") 
#SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -pthread -O3 -march=native") 
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -pthread -mcpu=cortex-a9 -mtune=cortex-a9 -march=armv7-a -mfpu=vfpv3-d16 -mfloat-abi=hard")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -mcpu=cortex-a9 -mtune=cortex-a9 -march=armv7-a -mfpu=vfpv3-d16 -mfloat-abi=hard")

#set(BOARD_EMBUILD_PATH /home/jiangdaihong/datajdh/jiangdaihong/board_workspace/px30_project/embuild_opencv_opencl_encrypt)
set(BOARD_EMBUILD_PATH /home/likun/Rockchip_px30)
set(CMAKE_CXX_COMPILER ${BOARD_EMBUILD_PATH}/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
set(CMAKE_C_COMPILER ${BOARD_EMBUILD_PATH}/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)

#px30
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--as-needed -fPIC -pthread -mcpu=cortex-a35 -mtune=cortex-a35 -march=armv8-a -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-Wl,--as-needed -fPIC -pthread -mcpu=cortex-a35 -mtune=cortex-a35 -march=armv8-a -O3 ")


#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -pthread -O3 -mcpu=cortex-a7 -mtune=cortex-a7 -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -O3 -mcpu=cortex-a7 -mtune=cortex-a7 -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard")

#pc
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -O3 -march=native") 
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -O3 -march=native")  


# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
#   message(STATUS "Using flag -std=c++11.")
#elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#   add_definitions(-DCOMPILEDWITHC0X)
#   message(STATUS "Using flag -std=c++0x.")
#else()
#   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
LIST(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(UNIX)

# For building the CHOLMOD / CSPARSE solvers
#FIND_PACKAGE(BLAS REQUIRED)
#FIND_PACKAGE(LAPACK REQUIRED)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND G2O_USE_OPENMP)
  SET (G2O_OPENMP 1)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)

# Compiler specific options for gcc
#SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native") 
#SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native") 
#    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -pthread -mcpu=cortex-a9 -mtune=cortex-a9 -march=armv7-a -mfpu=vfpv3-d16 -mfloat-abi=hard")
#    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -pthread -mcpu=cortex-a9 -mtune=cortex-a9 -march=armv7-a -mfpu=vfpv3-d16 -mfloat-abi=hard")



# activate warnings !!!
SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3
set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../eigen3/)
SET(G2O_EIGEN3_INCLUDE ${PROJECT_SOURCE_DIR}/../eigen3/)
#set(EIGEN3_INCLUDE_DIR /usr/include/eigen3/)
#SET(G2O_EIGEN3_INCLUDE /usr/include/eigen3/)
#FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
set(EIGEN3_FOUND TRUE)
IF(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
ENDIF(EIGEN3_FOUND)
message("${EIGEN3_INCLUDE_DIR}")
# Generate config.h
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
#configure_file(config.h.in ${g2o_SOURCE_DIR}/config.h)

set(OpenCV_DIR /opt/opencv2.4.9/share/OpenCV)
find_package( OpenCV REQUIRED )
message("opencv ${OpenCV_VERSION}")
message("${OpenCV_INCLUDE_DIRS}")
message("${OpenCV_LIBS}")

# Set up the top-level include directories
INCLUDE_DIRECTORIES(
${g2o_SOURCE_DIR}
${g2o_SOURCE_DIR}/..
${g2o_SOURCE_DIR}/core
${g2o_SOURCE_DIR}/types
${g2o_SOURCE_DIR}/stuff 
${EIGEN3_INCLUDE_DIR}
${OpenCV_LIBS}
${G2O_EIGEN3_INCLUDE})

# Include the subdirectories
ADD_LIBRARY(${LIB_NAME} ${G2O_LIB_TYPE}
#types
g2o/types/types_sba.h
g2o/types/types_six_dof_expmap.h
g2o/types/types_sba.cpp
g2o/types/types_six_dof_expmap.cpp
g2o/types/types_seven_dof_expmap.cpp
g2o/types/types_seven_dof_expmap.h
g2o/types/se3quat.h
g2o/types/se3_ops.h
g2o/types/se3_ops.hpp
g2o/types/edge_se2_XYZ.h
g2o/types/se2.h
g2o/types/vertex_se2.cpp
g2o/types/vertex_se2.h
g2o/types/g2o_types_slam2d_api.h

#core
g2o/core/base_edge.h
g2o/core/eigen_types.h
g2o/core/base_binary_edge.h
g2o/core/hyper_graph_action.cpp
g2o/core/base_binary_edge.hpp
g2o/core/hyper_graph_action.h
g2o/core/base_multi_edge.h           
g2o/core/hyper_graph.cpp
g2o/core/base_multi_edge.hpp         
g2o/core/hyper_graph.h
g2o/core/base_unary_edge.h          
g2o/core/linear_solver.h
g2o/core/base_unary_edge.hpp         
g2o/core/marginal_covariance_cholesky.cpp
g2o/core/base_vertex.h               
g2o/core/marginal_covariance_cholesky.h
g2o/core/base_vertex.hpp             
g2o/core/matrix_structure.cpp
g2o/core/batch_stats.cpp             
g2o/core/matrix_structure.h
g2o/core/batch_stats.h               
g2o/core/openmp_mutex.h
g2o/core/block_solver.h              
g2o/core/block_solver.hpp            
g2o/core/parameter.cpp               
g2o/core/parameter.h                 
g2o/core/cache.cpp                   
g2o/core/cache.h
g2o/core/optimizable_graph.cpp       
g2o/core/optimizable_graph.h         
g2o/core/solver.cpp                  
g2o/core/solver.h
g2o/core/creators.h                 
g2o/core/optimization_algorithm_factory.cpp
g2o/core/estimate_propagator.cpp     
g2o/core/optimization_algorithm_factory.h
g2o/core/estimate_propagator.h       
g2o/core/factory.cpp                 
g2o/core/optimization_algorithm_property.h
g2o/core/factory.h                   
g2o/core/sparse_block_matrix.h
g2o/core/sparse_optimizer.cpp  
g2o/core/sparse_block_matrix.hpp
g2o/core/sparse_optimizer.h
g2o/core/hyper_dijkstra.cpp 
g2o/core/hyper_dijkstra.h
g2o/core/parameter_container.cpp     
g2o/core/parameter_container.h
g2o/core/optimization_algorithm.cpp 
g2o/core/optimization_algorithm.h
g2o/core/optimization_algorithm_with_hessian.cpp 
g2o/core/optimization_algorithm_with_hessian.h
g2o/core/optimization_algorithm_levenberg.cpp 
g2o/core/optimization_algorithm_levenberg.h
g2o/core/jacobian_workspace.cpp 
g2o/core/jacobian_workspace.h
g2o/core/robust_kernel.cpp 
g2o/core/robust_kernel.h
g2o/core/robust_kernel_factory.cpp
g2o/core/robust_kernel_factory.h
g2o/core/robust_kernel_impl.cpp 
g2o/core/robust_kernel_impl.h
#stuff
g2o/stuff/string_tools.h
g2o/stuff/color_macros.h 
g2o/stuff/macros.h
g2o/stuff/timeutil.cpp
g2o/stuff/misc.h
g2o/stuff/timeutil.h
g2o/stuff/os_specific.c    
g2o/stuff/os_specific.h
g2o/stuff/string_tools.cpp
g2o/stuff/property.cpp       
g2o/stuff/property.h       
)
