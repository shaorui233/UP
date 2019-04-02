%module pycheetah
%include "carrays.i"
%array_class(double, doubleArray);
%{
    extern int step(double* in_jpos, double* in_jvel, double* out_config, double* out_config_vel, bool reset);
%}

extern int step(double* in_jpos, double* in_jvel,double* out_config, double* out_config_vel, bool reset);
