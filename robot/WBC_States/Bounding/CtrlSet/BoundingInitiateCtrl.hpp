#ifndef BOUNDING_INITIATION_CTRL
#define BOUNDING_INITIATION_CTRL

#include <WBC_States/Controller.hpp>
#include <ParamHandler/ParamHandler.hpp>

template <typename T> class ContactSpec;
template <typename T> class WBIC;
template <typename T> class WBIC_ExtraData;
template <typename T> class StateProvider;
template <typename T> class BoundingTest;

template <typename T>
class BoundingInitiateCtrl: public Controller<T>{
    public:
        BoundingInitiateCtrl(const FloatingBaseModel<T>* );
        virtual ~BoundingInitiateCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & category_name);
        virtual void SetTestParameter(const std::string & test_file);

    protected:
        std::vector<T> _Kp_joint, _Kd_joint;

        BoundingTest<T>* _bounding_test;
        DVec<T> _des_jpos; 
        DVec<T> _des_jvel; 

        T _end_time;
        int _dim_contact;

        Task<T>* _body_posture_task;

        ContactSpec<T>* _fr_contact;
        ContactSpec<T>* _fl_contact;
        ContactSpec<T>* _hr_contact;
        ContactSpec<T>* _hl_contact;
        WBIC<T>* _wbic;
        WBIC_ExtraData<T>* _wbic_data;

        T _target_body_height;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wbic(DVec<T> & gamma);

        T _ctrl_start_time;
        ParamHandler* _param_handler;
        StateProvider<T>* _sp;
};

#endif
