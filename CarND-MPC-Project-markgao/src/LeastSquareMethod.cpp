#include "LeastSquareMethod.h"

/**
 * @brief Fit polynomial using Least Square Method.
 * 
 * @param X X-axis coordinate vector of sample data.
 * @param Y Y-axis coordinate vector of sample data.
 * @param orders Fitting order which should be larger than zero. 
 * @return Eigen::VectorXf Coefficients vector of fitted polynomial.
 */
Eigen::VectorXd FitterLeastSquareMethod(vector<double> &X, vector<double> &Y, uint8_t orders)
{
    // abnormal input verification
    if (X.size() < 2 || Y.size() < 2 || X.size() != Y.size() || orders < 1)
        exit(EXIT_FAILURE);

    // map sample data from STL vector to eigen vector
    Eigen::Map<Eigen::VectorXd> sampleX(X.data(), X.size());
    Eigen::Map<Eigen::VectorXd> sampleY(Y.data(), Y.size());

    Eigen::MatrixXd mtxVandermonde(X.size(), orders + 1);  // Vandermonde matrix of X-axis coordinate vector of sample data
    Eigen::VectorXd colVandermonde = sampleX;              // Vandermonde column

    // construct Vandermonde matrix column by column
    for (size_t i = 0; i < orders + 1; ++i)
    {
        if (0 == i)
        {
            mtxVandermonde.col(0) = Eigen::VectorXd::Constant(X.size(), 1, 1);
            continue;
        }
        if (1 == i)
        {
            mtxVandermonde.col(1) = colVandermonde;
            continue;
        }
        colVandermonde = colVandermonde.array()*sampleX.array();
        mtxVandermonde.col(i) = colVandermonde;
    }

    // calculate coefficients vector of fitted polynomial
    Eigen::VectorXd result = (mtxVandermonde.transpose()*mtxVandermonde).inverse()*(mtxVandermonde.transpose())*sampleY;

    return result;
}
