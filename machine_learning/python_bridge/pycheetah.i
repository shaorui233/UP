%module pycheetah
%include "carrays.i"
%array_class(double, doubleArray);
%{
    extern int step(double* in_jpos, double* in_jvel);
%}

extern int step(double* in_jpos, double* in_jvel);
