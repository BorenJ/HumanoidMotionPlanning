#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <cassert>
#include "coin-or/IpIpoptApplication.hpp"
#include "hs071_nlp.hpp"
#include "nao.hpp"
using namespace Ipopt;


int main(
   int    /*argv*/,
   char** /*argc*/
)
 {   
    // NaoRobot nao;  
    com_goal[0] = 20.15;
    com_goal[1] = 13.4;
    q_init << 0, -20, 0, 49, -29, 0, -20, 0, 49, -29, 0;

    SmartPtr<TNLP> mynlp = new HS071_NLP();

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetStringValue("derivative_test", "first-order");
    app->Options()->SetStringValue("gradient_approximation", "finite-difference-values");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetStringValue("jacobian_approximation", "finite-difference-values");

    app->Options()->SetNumericValue("tol", 1e-2);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");

    ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Solve_Succeeded )
    {
        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
        return (int) status;
    }

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);

    if( status == Solve_Succeeded )
    {
        std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
    }
    else
    {
        std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
    }

    return (int) status;

}