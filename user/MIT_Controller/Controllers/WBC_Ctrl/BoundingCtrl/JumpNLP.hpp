#ifndef JUMP_NLP
#define JUMP_NLP

#ifdef IPOPT_OPTION

#include <IpTNLP.hpp>
#include "cppTypes.h"

using namespace Ipopt;

template<typename T>
class JumpNLP: public TNLP{
  public:
    JumpNLP();
    virtual ~JumpNLP(){}

    // (x, z, theta, xdot, zdot, theta_dot)
    int n_state = 6;
    int stance_tick = 13;
    int swing_tick = 15;
    int n_step;

    // state * (num_step + 1) + 
    // front reaction force (x, z) + 
    // hind reaction force (x, z) +
    // hind foot location
    int n_opt = 2;

    // initial + 
    // dynamics + 
    // front foot friction cone (-mu*fr_z < fr_x < mu*fr_z) + 
    // hind foot friction cone (-mu*fr_z < fr_x < mu*fr_z) + 
    // leg length
    int n_constr = 1;

    T v_des = 1.;
    T w_des = 1.0;
    T theta_des = -0.05;
    T dt = 0.01;

    Vec3<T> ini_pos; // x, z, theta
    Vec3<T> ini_vel; // d(x, z, theta)
    Vec2<T> Fr_foot_loc; // foot(x, z)

  protected:
    T _half_body = 0.19;
    T _leg_max = 0.3;
    T _mu = 0.5;
    T _grav = 9.81;
    T _body_mass = 9.;
    T _Iy = 0.271426;

    T _min_height = 0.18;

    void _print_problem_setup();


  public:
    /* --------------------------IPOPT Methods-------------------------- */

    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the nlp */
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(Ipopt::Index n, Number* x_l, Number* x_u,
                                 Ipopt::Index m, Number* g_l, Number* g_u);

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Number* x,
                                    bool init_z, Number* z_L, Number* z_U,
                                    Ipopt::Index m, bool init_lambda,
                                    Number* lambda);

    /** Method to return the objective value */
    virtual bool eval_f(Ipopt::Index n, const Number* x, bool new_x, Number& obj_value);

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(Ipopt::Index n, const Number* x, bool new_x, Number* grad_f);

    /** Method to return the constraint residuals */
    virtual bool eval_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Number* g);

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g(Ipopt::Index n, const Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac,
                            Ipopt::Index* iRow, Ipopt::Index *jCol,
                            Number* values);

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    virtual bool eval_h(Ipopt::Index n, const Number* x, bool new_x,
                        Number obj_factor, Ipopt::Index m, const Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Number* values);

    //@}

    /** @name Solution Methods */
    //@{
    /** This method is called when the algorithm is complete 
     * so the TNLP can store/write the solution */
    virtual void finalize_solution(SolverReturn status,
                                   Ipopt::Index n, const Number* x,
                                   const Number* z_L, const Number* z_U,
                                   Ipopt::Index m, const Number* g,
                                   const Number* lambda, Number obj_value,
                                   const IpoptData* ip_data,
                                   IpoptCalculatedQuantities* ip_cq);
};

#endif // IPOPT_OPTION

#endif
