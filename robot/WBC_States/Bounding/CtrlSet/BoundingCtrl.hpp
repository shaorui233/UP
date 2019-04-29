#ifndef BOUNDING_CTRL
#define BOUNDING_CTRL

#include <WBC_States/Controller.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include "ImpulseCurve.hpp"

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

        bool _b_front_contact;
        bool _b_hind_contact;


        T _end_time;
        T _swing_time;
        T _stance_time;
        int _dim_contact;

        T _ctrl_start_time;
        T _front_start_time;
        T _hind_start_time;

        T _front_time;
        T _hind_time;


        Task<T>* _abd_joint_task;
        Task<T>* _local_roll_task;
        Task<T>* _body_xy_task;
        Task<T>* _body_ori_task;
        Task<T>* _body_ryrz_task;

        Task<T>* _fr_leg_height_task;
        Task<T>* _fl_leg_height_task;
        Task<T>* _hr_leg_height_task;
        Task<T>* _hl_leg_height_task;

        Task<T>* _fr_foot_local_task;
        Task<T>* _fl_foot_local_task;
        Task<T>* _hr_foot_local_task;
        Task<T>* _hl_foot_local_task;

        Task<T>* _fr_abduction_task;
        Task<T>* _fl_abduction_task;
        Task<T>* _hr_abduction_task;
        Task<T>* _hl_abduction_task;

        Vec3<T> _fr_abduction_pos;
        Vec3<T> _fl_abduction_pos;
        Vec3<T> _hr_abduction_pos;
        Vec3<T> _hl_abduction_pos;

        Vec3<T> _fr_foot_pos;
        DVec<T> _fr_foot_vel;
        DVec<T> _fr_foot_acc;

        Vec3<T> _fl_foot_pos;
        DVec<T> _fl_foot_vel;
        DVec<T> _fl_foot_acc;

        Vec3<T> _hr_foot_pos;
        DVec<T> _hr_foot_vel;
        DVec<T> _hr_foot_acc;

        Vec3<T> _hl_foot_pos;
        DVec<T> _hl_foot_vel;
        DVec<T> _hl_foot_acc;

        DVec<T> _Fr_des;
        DVec<T> _Fr_result;

        ContactSpec<T>* _fr_contact;
        ContactSpec<T>* _fl_contact;
        ContactSpec<T>* _hr_contact;
        ContactSpec<T>* _hl_contact;
        WBIC<T>* _wbic;
        WBIC_ExtraData<T>* _wbic_data;

        T _target_leg_height;
        ImpulseCurve<T> _z_impulse;

        void _body_task_setup();
        void _leg_task_setup();
        void _contact_update();
        void _compute_torque_wbic(DVec<T> & gamma);

        void _StatusCheck();
        void _setupTaskAndContactList();

        std::string _folder_name;
        ParamHandler* _param_handler;
        StateProvider<T>* _sp;
};

#endif
