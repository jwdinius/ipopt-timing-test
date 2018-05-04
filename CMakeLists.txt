cmake_minimum_required (VERSION 2.6)
project (test_ipopt_install)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "-g -O3 -march=native -Wall -Wextra")

enable_testing()

include_directories(${PROJECT_SOURCE_DIR}/include)

# Find Eigen
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

# Find CppAD
find_package(CppAD REQUIRED)
include_directories(${CPPAD_INCLUDE_DIRS})

# Find IPOPT
find_package(IPOPT REQUIRED)
include_directories(${IPOPT_INCLUDE_DIRS})
add_definitions(${IPOPT_DEFINITIONS})

# Build mpc
add_library(mpc SHARED src/mpc.cpp)
target_link_libraries(mpc ${EIGEN_LIBRARY} ${CPPAD_LIBRARIES} ${IPOPT_LIBRARIES})

# Build test_ipopt_install
add_executable(test_ipopt_install src/test_install.cpp)
target_link_libraries(test_ipopt_install mpc)
add_test(NAME test_ipopt_install COMMAND test_ipopt_install)

# Run 'make check' to build all tests and then run them
add_custom_target(check test_ipopt_test COMMAND ${CMAKE_CTEST_COMMAND} DEPENDS test_ipopt_install)

# Set up installation targets
install(FILES include/mpc.hpp DESTINATION include)
install(TARGETS mpc DESTINATION lib)

