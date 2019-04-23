#ifndef WBDC_TROT_FULL_CONTACT_BODY_CTRL
#define WBDC_TROT_FULL_CONTACT_BODY_CTRL

#include <WBC_States/Controller.hpp>
#include <ParamHandler/ParamHandler.hpp>

template <typename T> class ContactSpec;
template <typename T> class WBDC;
template <typename T> class WBDC_ExtraData;
template <typename T> class StateProvider;
template <typename T> class WBDCTrotTest;

template <typename T>
class WBDCVM_FullContactCtrl: public Controller<T>{
    public:
        WBDCVM_FullContactCtrl(WBDCTrotTest<T> *, const FloatingBaseModel<T>* );
        virtual ~WBDCVM_FullContactCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & category_name);
        virtual void SetTestParameter(const std::string & test_file);

    protected:
        std::vector<T> _Kp_joint, _Kd_joint;

        WBDCTrotTest<T>* _trot_test;
        DVec<T> _des_jpos; 
        DVec<T> _des_jvel; 

        T _end_time;
        int _dim_contact;

        Task<T>* _body_posture_task;

        ContactSpec<T>* _fr_contact;
        ContactSpec<T>* _fl_contact;
        ContactSpec<T>* _hr_contact;
        ContactSpec<T>* _hl_contact;
        WBDC<T>* _wbdc;
        WBDC_ExtraData<T>* _wbdc_data;

        T _target_body_height;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wbdc(DVec<T> & gamma);

        T _ctrl_start_time;
        ParamHandler* _param_handler;
        StateProvider<T>* _sp;
};

#endif
