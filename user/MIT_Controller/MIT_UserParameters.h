#ifndef PROJECT_MITUSERPARAMETERS_H
#define PROJECT_MITUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class MIT_UserParameters : public ControlParameters {
public:
  MIT_UserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(cmpc_gait),
        INIT_PARAMETER(use_wbc),
        INIT_PARAMETER(wbc_base_Fr_weight),
        INIT_PARAMETER(Kp_body),
        INIT_PARAMETER(Kd_body),
        INIT_PARAMETER(Kp_ori),
        INIT_PARAMETER(Kd_ori),
        INIT_PARAMETER(Kp_foot),
        INIT_PARAMETER(Kd_foot),
        INIT_PARAMETER(Kp_joint),
        INIT_PARAMETER(Kd_joint),
        INIT_PARAMETER(use_jcqp),
        INIT_PARAMETER(jcqp_max_iter),
        INIT_PARAMETER(jcqp_rho),
        INIT_PARAMETER(jcqp_sigma),
        INIT_PARAMETER(jcqp_alpha),
        INIT_PARAMETER(jcqp_terminate)
  {}

  DECLARE_PARAMETER(double, cmpc_gait);
  DECLARE_PARAMETER(double, use_wbc);
  DECLARE_PARAMETER(double, wbc_base_Fr_weight);

  DECLARE_PARAMETER(Vec3<double>, Kp_body);
  DECLARE_PARAMETER(Vec3<double>, Kd_body);

  DECLARE_PARAMETER(Vec3<double>, Kp_ori);
  DECLARE_PARAMETER(Vec3<double>, Kd_ori);

  DECLARE_PARAMETER(Vec3<double>, Kp_foot);
  DECLARE_PARAMETER(Vec3<double>, Kd_foot);

  DECLARE_PARAMETER(Vec3<double>, Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint);

  DECLARE_PARAMETER(double, use_jcqp);
  DECLARE_PARAMETER(double, jcqp_max_iter);
  DECLARE_PARAMETER(double, jcqp_rho);
  DECLARE_PARAMETER(double, jcqp_sigma);
  DECLARE_PARAMETER(double, jcqp_alpha);
  DECLARE_PARAMETER(double, jcqp_terminate);
};

#endif //PROJECT_MITUSERPARAMETERS_H
