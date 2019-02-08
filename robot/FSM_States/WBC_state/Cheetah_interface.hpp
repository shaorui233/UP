#ifndef Cheetah_INTERFACE_H
#define Cheetah_INTERFACE_H

#include <cppTypes.h>
#include "Cheetah_DynaCtrl_Definition.h"
#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>

template <typename T> class Test;
template <typename T> class Cheetah_StateProvider;

template <typename T>
class Cheetah_interface{
    public:
        Cheetah_interface(FloatingBaseModel<T>* robot);
        ~Cheetah_interface();

        void GetCommand(const Cheetah_Data<T>* data, LegControllerCommand<T> * command);

    private:
        FloatingBaseModel<T> * _robot;
        FBModelState<T> _state;
        int count_;
        int waiting_count_;
        T running_time_;

        Test<T>* _test;

        void _ParameterSetting();
        bool _Initialization(const Cheetah_Data<T>* , LegControllerCommand<T>* );
        
        Cheetah_StateProvider<T>* sp_;
};

#endif
