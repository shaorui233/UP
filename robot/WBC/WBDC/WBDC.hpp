#ifndef WHOLE_BODY_DYNAMIC_CONTROL_H
#define WHOLE_BODY_DYNAMIC_CONTROL_H

#include <WBC/WBC.hpp>
#include <WBC/ContactSpec.hpp>
#include <WBC/Task.hpp>
#include <Utilities/Utilities_print.h>
#include <Goldfarb_Optimizer/QuadProg++.hh>

template<typename T>
class WBDC_ExtraData{
    public:
        // Output
        DVec<T> _opt_result;
        DVec<T> _qddot;
        DVec<T> _Fr;

        // Input
        DVec<T> _W_contact;
        DVec<T> _W_task;
        DVec<T> _W_rf;
        DVec<T> _contact_pt_acc;

        WBDC_ExtraData(){}
        ~WBDC_ExtraData(){}
};

template<typename T>
class WBDC: public WBC<T>{
    public:
        WBDC(size_t num_qdot, 
            const std::vector<ContactSpec<T>* > & contact_list,
            const std::vector<Task<T>* > & task_list);
        virtual ~WBDC(){}

        virtual void UpdateSetting(const DMat<T> & A,
                const DMat<T> & Ainv,
                const DVec<T> & cori,
                const DVec<T> & grav,
                void* extra_setting = NULL);

        virtual void MakeTorque(
                DVec<T> & cmd,
                void* extra_input = NULL);

    private:
        std::vector<ContactSpec<T> *> _contact_list;
        std::vector<Task<T> *> _task_list;

        void _SetInEqualityConstraint();
        void _ContactBuilding();

        void _GetSolution(const DVec<T> & qddot, DVec<T> & cmd);
        bool _CheckNullSpace(const DMat<T> & Npre);
        void _SetCost();

        size_t _dim_opt; // Contact pt delta, First task delta, reaction force
        size_t _dim_eq_cstr; // equality constraints
        
        size_t _dim_first_task; // first task dimension
        size_t _dim_rf; // inequality constraints
        size_t _dim_Uf;

        WBDC_ExtraData<T>* _data;

        GolDIdnani::GVect<double> z;
        // Cost
        GolDIdnani::GMatr<double> G;
        GolDIdnani::GVect<double> g0;

        // Equality
        GolDIdnani::GMatr<double> CE;
        GolDIdnani::GVect<double> ce0;

        // Inequality
        GolDIdnani::GMatr<double> CI;
        GolDIdnani::GVect<double> ci0;

        DMat<T> _dyn_CE;
        DVec<T> _dyn_ce0;
        DMat<T> _dyn_CI;
        DVec<T> _dyn_ci0;

        DMat<T> _eye;

        DMat<T> _S_delta;
        DMat<T> _Uf;
        DVec<T> _Uf_ieq_vec;

        DMat<T> _Ja;
        DVec<T> _JaDotQdot;
        DVec<T> _xa_ddot;

        DMat<T> _Jc;
        DVec<T> _JcDotQdot;

        DMat<T> _B;
        DVec<T> _c;
        DVec<T> task_cmd_;

        DMat<T> _Sf; //floating base
        void _PrintDebug(T i) {
            (void)i;
            //printf("[WBDC] %f \n", i);
        }
};

#endif
