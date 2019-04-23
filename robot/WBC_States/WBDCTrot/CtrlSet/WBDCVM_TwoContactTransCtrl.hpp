#ifndef PLANNED_TROT_TWO_CONTACT_TRANSITION_CHEETAH
#define PLANNED_TROT_TWO_CONTACT_TRANSITION_CHEETAH

#include <WBC_States/Controller.hpp>
#include <ParamHandler/ParamHandler.hpp>

template <typename T> class ContactSpec;
template <typename T> class WBDC;
template <typename T> class WBDC_ExtraData;
template <typename T> class StateProvider;
template <typename T> class WBDCTrotTest;

template <typename T>
class WBDCVM_TwoContactTransCtrl: public Controller<T>{
    public:
        WBDCVM_TwoContactTransCtrl(WBDCTrotTest<T> * test, const FloatingBaseModel<T>* robot, 
                size_t cp1, size_t cp2, int transit_dir);
        virtual ~WBDCVM_TwoContactTransCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);
        virtual void SetTestParameter(const std::string & test_file);

    protected:
        void _SetContact(const size_t & cp_idx, const T & upper_lim, 
                const T & rf_weight, const T & rf_weight_z, const T & foot_weight);

        std::vector<T> _Kp_joint, _Kd_joint;
        WBDCTrotTest<T> * _trot_test;
        size_t _cp1, _cp2;
        int _transit_dir; // 1: swing start, -1: swing end
 
        Task<T>* _body_posture_task;

        ContactSpec<T>* _fr_contact;
        ContactSpec<T>* _fl_contact;
        ContactSpec<T>* _hr_contact;
        ContactSpec<T>* _hl_contact;

        WBDC<T>* _wbdc;
        WBDC_ExtraData<T>* _wbdc_data;

        DVec<T> _ini_jpos;

        DVec<T> _des_jpos;
        DVec<T> _des_jvel;

        T _end_time;
        T _target_body_height;
        Vec3<T> _ini_body_pos;
        
        T _max_rf_z;
        T _min_rf_z;
        int _dim_contact;
        T _ctrl_start_time;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wbdc(DVec<T> & gamma);

        ParamHandler* _param_handler;
        StateProvider<T>* _sp;
        std::string _test_file_name;
};

#endif
