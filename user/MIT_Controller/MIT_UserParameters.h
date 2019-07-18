#ifndef PROJECT_MITUSERPARAMETERS_H
#define PROJECT_MITUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class MIT_UserParameters : public ControlParameters {
public:
  MIT_UserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(cmpc_gait),
        INIT_PARAMETER(use_wbc),
        INIT_PARAMETER(wbc_base_Fr_weight)
      {}

  DECLARE_PARAMETER(double, cmpc_gait);
  DECLARE_PARAMETER(double, use_wbc);
  DECLARE_PARAMETER(double, wbc_base_Fr_weight);
};

#endif //PROJECT_MITUSERPARAMETERS_H
