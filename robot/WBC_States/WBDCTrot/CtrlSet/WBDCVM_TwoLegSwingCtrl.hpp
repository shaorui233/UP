#ifndef WBDC_VM_TWO_SWING_CHEETAH
#define WBDC_VM_TWO_SWING_CHEETAH

#include <WBC_States/Controller.hpp>
#include <Utilities/BSplineBasic.h>

template <typename T> class ContactSpec;
template <typename T> class WBDC;
template <typename T> class WBDC_ExtraData;
template <typename T> class StateProvider;
template <typename T> class WBDCTrotTest;

template <typename T>
class WBDCVM_TwoLegSwingCtrl: public Controller<T>{
    public:
        WBDCVM_TwoLegSwingCtrl(WBDCTrotTest<T> * test, 
                const FloatingBaseModel<T>* , size_t cp1, size_t cp2);
        virtual ~WBDCVM_TwoLegSwingCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);
        virtual void SetTestParameter(const std::string & test_file);

    protected:
        void _GetSinusoidalSwingTrajectory(
                const Vec3<T> & ini, const Vec3<T> & fin, const T & t, 
                Vec3<T> & pos_des, DVec<T> & vel_des, DVec<T> & acc_des);

        void _GetBsplineSwingTrajectory(const T & t, BS_Basic<double, 3, 3, 1, 2, 2> & spline,
                Vec3<T> & pos_des, DVec<T> & vel_des, DVec<T> & acc_des);
        void _SetBspline(const Vec3<T> & st_pos, const Vec3<T> & des_pos, 
                BS_Basic<double, 3, 3, 1, 2, 2> & spline);
        void _SetContact(const size_t & cp_idx, const T & upper_lim, 
                const T & rf_weight, const T & rf_weight_z, const T & foot_weight);

        void _updateContactAcc(const size_t & cp_idx, const DVec<T>& cmd);

        WBDCTrotTest<T>* _trot_test;
        size_t _cp1, _cp2;
        Vec3<T> _default_target_foot_loc_1;
        Vec3<T> _default_target_foot_loc_2;
        Vec3<T> _landing_offset;
        T _swing_height;
        Vec3<T> _prev_ori_command;

        Task<T>* _cp_pos_task1;
        Task<T>* _cp_pos_task2;

        Vec3<T> _foot_pos_ini1;
        Vec3<T> _target_loc1;
        Vec3<T> _foot_pos_des1;
        DVec<T> _foot_vel_des1;
        DVec<T> _foot_acc_des1;

        Vec3<T> _foot_pos_ini2;
        Vec3<T> _target_loc2;
        Vec3<T> _foot_pos_des2;
        DVec<T> _foot_vel_des2;
        DVec<T> _foot_acc_des2;

        Task<T>* _body_posture_task;

        ContactSpec<T>* _fr_contact;
        ContactSpec<T>* _fl_contact;
        ContactSpec<T>* _hr_contact;
        ContactSpec<T>* _hl_contact;

        WBDC<T>* _wbdc;
        WBDC_ExtraData<T>* _wbdc_data;

        DVec<T> base_pos_ini_;
        Vec3<T> ini_base_pos_;

        DVec<T> _des_jpos;
        DVec<T> _des_jvel;

        T _end_time;
        T _body_height_cmd;
        T ini_body_height_;
        Vec3<T> _ini_body_pos;
        Vec3<T> _ini_body_target;
        
        int _dim_contact;
        T _ctrl_start_time;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wbdc(DVec<T> & gamma);

        StateProvider<T>* _sp;
        T _dir_command[2];
        std::string _test_file_name;
        BS_Basic<double, 3, 3, 1, 2, 2> _foot_traj_1;
        BS_Basic<double, 3, 3, 1, 2, 2> _foot_traj_2;
};
#endif
