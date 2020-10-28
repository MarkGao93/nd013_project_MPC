/**************************************************************************************************
 * @ File: main.cpp
 * @ Author: MarkGao
 * @ Date: 2020-05-15
 * @ Email: 819699632@qq.com
 * @ Version: 1.0
 * @ Description:
**************************************************************************************************/
#include <vector>
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "matplotlibcpp.h"
#include "MPC.h"

namespace plt = matplotlibcpp;

using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;



/**************************************************************************************************
 * @ Function: main()
 * @ Description:      // Description of the function
 * @ Calls:                       // Functions being called by this function
 * @ Input:                       // Description of input parameters, include the function, value range of each parameter
 * @ Output:                    // Description of output parameters
 * @ Return:                    // Description of return value
 * @ Others:
**************************************************************************************************/
int main()
{
    MPC mpc;
    int iters = 100;

    VectorXd ptsx(2);
    VectorXd ptsy(2);
    ptsx << -100, 100;
    ptsy << -1, 1;



    // TODO: fit a polynomial to the above x and y coordinates
    auto coeffs = polyfit(ptsx, ptsy, 1);    
//    cout<<coeffs.rows()<<endl;
//    cout<<coeffs.cols()<<endl;
    cout<<"a0 = "<<coeffs[0]<<endl;
    cout<<"a1 = "<<coeffs[1]<<endl;

    // NOTE: free feel to play around with these
    double x = -1;
    double y = -1;    // 10
    double psi = 0;
    double v = 10;


    // TODO: calculate the cross track error
    double cte = polyeval(coeffs, x) - y;
    cout<<"cte = "<<cte<<endl;


    // TODO: calculate the orientation error
    double epsi = psi - atan(coeffs[1]) ;
    cout<<"epsi = "<<epsi<<endl;
    cout<<"epsi = "<<atan(coeffs[1])<<endl;

    VectorXd state(6);
    state << x, y, psi, v, cte, epsi;

    vector<double> x_vals = {state[0]};
    vector<double> y_vals = {state[1]};
    vector<double> psi_vals = {state[2]};
    vector<double> v_vals = {state[3]};
    vector<double> cte_vals = {state[4]};
    vector<double> epsi_vals = {state[5]};
    vector<double> delta_vals = {};
    vector<double> a_vals = {};

    for(size_t i=0; i<iters; ++i)
    {
        cout<<"Iteration "<<i<<endl;

        auto vars = mpc.Solve(state, coeffs);
        cout<<"a0 = "<<coeffs[0]<<endl;
        cout<<"a1 = "<<coeffs[1]<<endl;

        x_vals.push_back(vars[0]);
        y_vals.push_back(vars[1]);
        psi_vals.push_back(vars[2]);
        v_vals.push_back(vars[3]);
        cte_vals.push_back(vars[4]);
        epsi_vals.push_back(vars[5]);

        delta_vals.push_back(vars[6]);
        a_vals.push_back(vars[7]);

        state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
        cout<<"x = "<<vars[0]<<endl;
        cout<<"y = "<<vars[1]<<endl;
        cout<<"psi = "<<vars[2]<<endl;
        cout<<"v = "<<vars[3]<<endl;
        cout<<"cte = "<<vars[4]<<endl;
        cout<<"epsi = "<<vars[5]<<endl;
//        cout<<"delta = "<<vars[6]<<endl;
//        cout<<"a = "<<vars[7]<<endl;
//        cout<<endl;
    }

    // Plot values
    // NOTE: feel free to play around with this.
    // It's useful for debugging!
//    plt::subplot(3, 1, 1);
//    plt::title("CTE");
//    plt::plot(cte_vals);
//    plt::subplot(3, 1, 2);
//    plt::title("Delta (Radians)");
//    plt::plot(delta_vals);
//    plt::subplot(3, 1, 3);
//    plt::title("Velocity");
//    plt::plot(v_vals);
    plt::figure();
    plt::title("path");
    plt::plot(x_vals, y_vals);

    plt::figure();
    plt::title("Velocity");
    plt::plot(v_vals);
//    plt::plot(v_vals);

    plt::show();
}
