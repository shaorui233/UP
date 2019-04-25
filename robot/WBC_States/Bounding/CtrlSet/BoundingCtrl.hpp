#ifndef BOUNDING_CTRL
#define BOUNDING_CTRL

#include <WBC_States/Controller.hpp>
#include <ParamHandler/ParamHandler.hpp>

template <typename T> class ContactSpec;
template <typename T> class WBIC;
template <typename T> class WBIC_ExtraData;
template <typename T> class StateProvider;
template <typename T> class BoundingTest;

template <typename T>
class BoundingCtrl: public Controller<T>{
    public:
        BoundingCtrl(BoundingTest<T> *, const FloatingBaseModel<T>* );
        virtual ~BoundingCtrl();

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

        bool _b_front_swing;
        bool _b_hind_swing;

        T _end_time;
        T _swing_time;
        T _stance_time;
        int _dim_contact;

        Task<T>* _body_xy_task;
        Task<T>* _body_ori_task;

        Task<T>* _fr_leg_height_task;
        Task<T>* _fl_leg_height_task;
        Task<T>* _hr_leg_height_task;
        Task<T>* _hl_leg_height_task;

        ContactSpec<T>* _fr_contact;
        ContactSpec<T>* _fl_contact;
        ContactSpec<T>* _hr_contact;
        ContactSpec<T>* _hl_contact;
        WBIC<T>* _wbic;
        WBIC_ExtraData<T>* _wbic_data;

        T _target_leg_height;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wbic(DVec<T> & gamma);

        T _ctrl_start_time;
        T _front_start_time;
        T _hind_start_time;

        ParamHandler* _param_handler;
        StateProvider<T>* _sp;
};

#endif
