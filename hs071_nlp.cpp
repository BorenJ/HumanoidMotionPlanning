// Copyright (C) 2005, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-16
#include "hs071_nlp.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
// #include "nao.hpp"
#include <cassert>
#include <iostream>
#include <memory>
using namespace Ipopt;

Eigen::VectorXd q_init(11);
double com_goal[2];
Number constrain[1];

// cost function
NaoRobot nao;

Number opt_fun(const Number* x, NaoRobot& nao, double* com_goal){
   // std::cout << "opt_fun: com_goal[0]  " << com_goal[0] << std::endl;
   // std::cout << "opt_fun: com_goal[1]  " << com_goal[1] << std::endl;
   auto [R, JointPos, FootCorner, EndeffLoc] = nao.NaoForwardKinematicsFullBody(JointAngle11to48(x));
   auto [Com, MassLoc] = nao.NaoCOMLoc(JointPos, R);
   // std::cout << "x0" << x[0] << std::endl;
   // std::cout << "x1" << x[1] << std::endl;
   // std::cout << "x2" << x[2] << std::endl;
   // std::cout << "x3" << x[3] << std::endl;
   // std::cout << "x4" << x[4] << std::endl;
   // std::cout << "x5" << x[5] << std::endl;
   // std::cout << "x6" << x[6] << std::endl;
   // std::cout << "x7" << x[7] << std::endl;
   // std::cout << "x8" << x[8] << std::endl;
   // std::cout << "x9" << x[9] << std::endl;
   // std::cout << "x10" << x[10] << std::endl;
   std::cout << "results " << Number( std::pow(Com(0) - com_goal[0], 2) + std::pow(Com(1) - com_goal[1], 2)) << std::endl;

   return Number( std::pow(Com(0) - com_goal[0], 2) + std::pow(Com(1) - com_goal[1], 2));
}

Number* opt_constrain(Number* constrain, const Number* x, NaoRobot& nao, const Eigen::VectorXd q_init){
   // std::cout << "constrainBegin:  " << std::endl;
   auto [R_con, JointPos_con, FootCorner_con, EndeffLoc_con] = nao.NaoForwardKinematicsFullBody(JointAngle11to48(x));
   auto [R, JointPos, FootCorner, EndeffLoc] = nao.NaoForwardKinematicsFullBody(JointAngle11to48(q_init));
   constrain[0] = Number((FootCorner_con[0] - FootCorner[0]).array().pow(2).sum());
   // std::cout << "constrainEnd:  " << constrain[0] << std::endl;
   return constrain;
}

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

// constructor
HS071_NLP::HS071_NLP(
   bool printiterate
) : printiterate_(printiterate)
{ }

// destructor
HS071_NLP::~HS071_NLP()
{}

// [TNLP_get_nlp_info]
// returns the size of the problem
bool HS071_NLP::get_nlp_info(
   Index&          n,
   Index&          m,
   Index&          nnz_jac_g,
   Index&          nnz_h_lag,
   IndexStyleEnum& index_style
)
{
   // The problem described in HS071_NLP.hpp has 4 variables, x[0] through x[3]
   n = 11;
   // one equality constraint and one inequality constraint
   m = 1;
   // in this example the jacobian is dense and contains 8 nonzeros
   nnz_jac_g = 11;
   // the Hessian is also dense and has 16 total nonzeros, but we
   // only need the lower left corner (since it is symmetric)
   nnz_h_lag = 0;
   // use the C style indexing (0-based)
   index_style = TNLP::C_STYLE;

   return true;
}
// [TNLP_get_nlp_info]

// [TNLP_get_bounds_info]
// returns the variable bounds
bool HS071_NLP::get_bounds_info(
   Index   n,
   Number* x_l,
   Number* x_u,
   Index   m,
   Number* g_l,
   Number* g_u
)
{
   // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
   // If desired, we could assert to make sure they are what we think they are.

   // the variables have lower bounds of 1
   for( Index i = 0; i < n; i++ )
   {
      x_l[i] = JointAngle48to11(nao.JointLim[0])[i];
   }

   // the variables have upper bounds of 5
   for( Index i = 0; i < n; i++ )
   {
      x_u[i] = JointAngle48to11(nao.JointLim[1])[i];
   }
   // the first constraint g1 has a lower bound of 25
   // g_l[0] = 0;
   // the first constraint g1 has NO upper bound, here we set it to 2e19.
   // Ipopt interprets any number greater than nlp_upper_bound_inf as
   // infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
   // is 1e19 and can be changed through ipopt options.
   // g_u[0] = 0;

   // the second constraint g2 is an equality constraint, so we set the
   // upper and lower bound to the same value
   g_l[0] = g_u[0] = 0;

   return true;
}
// [TNLP_get_bounds_info]

// [TNLP_get_starting_point]
// returns the initial point for the problem
bool HS071_NLP::get_starting_point(
   Index   n,
   bool    init_x,
   Number* x,
   bool    init_z,
   Number* z_L,
   Number* z_U,
   Index   m,
   bool    init_lambda,
   Number* lambda
)
{
   assert(init_x == true);
   assert(init_z == false);
   assert(init_lambda == false);

   // initialize to the given starting point
   for( Index i = 0; i < n; i++ )
   {
      x[i] = deg2rad(q_init)[i];
      // std::cout <<" start point" << x[i] << std::endl;
   }

   return true;
}
// [TNLP_get_starting_point]

// [TNLP_eval_f]
// returns the value of the objective function

bool HS071_NLP::eval_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number&       obj_value
)
{
   obj_value =  opt_fun(x, nao, com_goal);
   return true;
}
// [TNLP_eval_f]

// [TNLP_eval_grad_f]
// return the gradient of the objective function grad_{x} f(x)
bool HS071_NLP::eval_grad_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number*       grad_f
)
{
   return true;
}
// [TNLP_eval_grad_f]

// [TNLP_eval_g]
// return the value of the constraints: g(x)
bool HS071_NLP::eval_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Number*       g
)
{ 
   auto [R_con, JointPos_con, FootCorner_con, EndeffLoc_con] = nao.NaoForwardKinematicsFullBody(JointAngle11to48(x));
   auto [R, JointPos, FootCorner, EndeffLoc] = nao.NaoForwardKinematicsFullBody(JointAngle11to48(q_init));
   g[0] = Number((FootCorner_con[0] - FootCorner[0]).array().pow(2).sum());
   std::cout << "g[0]:     " << g[0] << std::endl;
   // g = opt_constrain(constrain, x, nao, q_init);
   return true;
}
// [TNLP_eval_g]

// [TNLP_eval_jac_g]
// return the structure or values of the Jacobian
bool HS071_NLP::eval_jac_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Index         nele_jac,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{  if( values == NULL )
   {
      // return the structure of the Jacobian
 
      // this particular Jacobian is dense
      iRow[0] = 0;
      jCol[0] = 0;
      iRow[1] = 0;
      jCol[1] = 1;
      iRow[2] = 0;
      jCol[2] = 2;
      iRow[3] = 0;
      jCol[3] = 3;
      iRow[4] = 0;
      jCol[4] = 4;
      iRow[5] = 0;
      jCol[5] = 5;
      iRow[6] = 0;
      jCol[6] = 6;
      iRow[7] = 0;
      jCol[7] = 7;
      iRow[8] = 0;
      jCol[8] = 8;
      iRow[9] = 0;
      jCol[9] = 9;
      iRow[10] = 0;
      jCol[10] = 10;
   }

   return true;
}
// [TNLP_eval_jac_g]

// [TNLP_finalize_solution]
void HS071_NLP::finalize_solution(
   SolverReturn               status,
   Index                      n,
   const Number*              x,
   const Number*              z_L,
   const Number*              z_U,
   Index                      m,
   const Number*              g,
   const Number*              lambda,
   Number                     obj_value,
   const IpoptData*           ip_data,
   IpoptCalculatedQuantities* ip_cq
) 
{
   // here is where we would store the solution to variables, or write to a file, etc
   // so we could use the solution.

   // For this example, we write the solution to the console
   std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
   for( Index i = 0; i < n; i++ )
   {
      std::cout << "x[" << i << "] = " << x[i] << std::endl;
   }

   std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
   for( Index i = 0; i < n; i++ )
   {
      std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
   }
   for( Index i = 0; i < n; i++ )
   {
      std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
   }

   std::cout << std::endl << std::endl << "Objective value" << std::endl;
   std::cout << "f(x*) = " << obj_value << std::endl;

   std::cout << std::endl << "Final value of the constraints:" << std::endl;
   for( Index i = 0; i < m; i++ )
   {
      std::cout << "g(" << i << ") = " << g[i] << std::endl;
   }
}
// [TNLP_finalize_solution]

// [TNLP_intermediate_callback]
bool HS071_NLP::intermediate_callback(
   AlgorithmMode              mode,
   Index                      iter,
   Number                     obj_value,
   Number                     inf_pr,
   Number                     inf_du,
   Number                     mu,
   Number                     d_norm,
   Number                     regularization_size,
   Number                     alpha_du,
   Number                     alpha_pr,
   Index                      ls_trials,
   const IpoptData*           ip_data,
   IpoptCalculatedQuantities* ip_cq
)
{
   if( !printiterate_ )
   {
      return true;
   }

   Number x[4];
   Number x_L_viol[4];
   Number x_U_viol[4];
   Number z_L[4];
   Number z_U[4];
   Number compl_x_L[4];
   Number compl_x_U[4];
   Number grad_lag_x[4];

   Number g[2];
   Number lambda[2];
   Number constraint_violation[2];
   Number compl_g[2];

   bool have_iter = get_curr_iterate(ip_data, ip_cq, false, 4, x, z_L, z_U, 2, g, lambda);
   bool have_viol = get_curr_violations(ip_data, ip_cq, false, 4, x_L_viol, x_U_viol, compl_x_L, compl_x_U, grad_lag_x, 2, constraint_violation, compl_g);

   printf("Current iterate:\n");
   printf("  %-12s %-12s %-12s %-12s %-12s %-12s %-12s\n", "x", "z_L", "z_U", "bound_viol", "compl_x_L", "compl_x_U", "grad_lag_x");
   for( int i = 0; i < 4; ++i )
   {
      if( have_iter )
      {
         printf("  %-12g %-12g %-12g", x[i], z_L[i], z_U[i]);
      }
      else
      {
         printf("  %-12s %-12s %-12s", "n/a", "n/a", "n/a");
      }
      if( have_viol )
      {
         printf(" %-12g %-12g %-12g %-12g\n", x_L_viol[i] > x_U_viol[i] ? x_L_viol[i] : x_U_viol[i], compl_x_L[i], compl_x_U[i], grad_lag_x[i]);
      }
      else
      {
         printf(" %-12s %-12s %-12s %-12s\n", "n/a", "n/a", "n/a", "n/a");
      }
   }

   printf("  %-12s %-12s %-12s %-12s\n", "g(x)", "lambda", "constr_viol", "compl_g");
   for( int i = 0; i < 2; ++i )
   {
      if( have_iter )
      {
         printf("  %-12g %-12g", g[i], lambda[i]);
      }
      else
      {
         printf("  %-12s %-12s", "n/a", "n/a");
      }
      if( have_viol )
      {
         printf(" %-12g %-12g\n", constraint_violation[i], compl_g[i]);
      }
      else
      {
         printf(" %-12s %-12s\n", "n/a", "n/a");
      }
   }

   return true;
}
// [TNLP_intermediate_callback]