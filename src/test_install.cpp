#include <iostream>
#include <mpc.hpp>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdlib>


int main(int argc, char** argv) {
    int N = 1;
    if (argc > 1) {
        N = atoi(argv[1]);
    }

    MPC mpc;
    Eigen::VectorXd coeffs(4);
    Eigen::VectorXd state(6);

    coeffs << -0.354214,
               0.002064,
               0.00111025,
               2.13429e-6;

    state << 0.0,
             0.0,
             0.0,
             19.7626,
            -0.354214,
            -0.002064;
    std::vector<double> elapsed_vector(N);
    for (auto i=0; i < N; i++) {
        auto start = std::chrono::system_clock::now();
        std::vector<double> optimal = mpc.Solve(state, coeffs);
        elapsed_vector[i] = std::chrono::duration_cast< std::chrono::duration<double> >(std::chrono::system_clock::now()-start).count();
        //std::cout << optimal[0] << ", " << optimal[1] << std::endl;
    }
    /*
     * expected output:
     * 0.00547966, 0.314773
     *
     * actual output:
     * 0.00547968, 0.314752
     *
     * result: pass
     */
    double mean = 0;
    for (auto i=0; i < N; i++) {
        mean += elapsed_vector[i];
    }
    mean /= static_cast<double>(N);

    double var = 0;
    for (auto i=0; i < N; i++) {
        auto delta = elapsed_vector[i] - mean;
        var = delta*delta;
    }
    double std = sqrt(var/static_cast<double>(N));

    std::cout << "----------------------TIMING STATS-----------------------" << std::endl;
    std::cout << "runs: " << N << ", mean: " << mean << ", stddev: " << std << std::endl;

    return 0;
}
