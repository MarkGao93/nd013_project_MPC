#ifndef LEAST_SQUARE_METHOD_H
#define LEAST_SQUARE_METHOD_H

#include "Eigen-3.3/Eigen/Dense"
#include <vector>
#include <string>
#include <stdint.h>

using namespace std;

/**
 * @brief Fit polynomial using Least Square Method.
 * 
 * @param X X-axis coordinate vector of sample data.
 * @param Y Y-axis coordinate vector of sample data.
 * @param orders Fitting order which should be larger than zero. 
 * @return Eigen::VectorXf Coefficients vector of fitted polynomial.
 */
Eigen::VectorXd FitterLeastSquareMethod(vector<double> &X, vector<double> &Y, uint8_t orders);

#endif
