#ifndef TROT_FULL_CONTACT_BODY_CTRL
#define TROT_FULL_CONTACT_BODY_CTRL

#include <WBC_States/Controller.hpp>
#include <ParamHandler/ParamHandler.hpp>

template <typename T> class ContactSpec;
template <typename T> class WBLC;
template <typename T> class WBLC_ExtraData;
template <typename T> class KinWBC;
template <typename T> class StateProvider;
template <typename T> class PlannedTrotTest;

namespace trot{
    template <typename T>
        class FullContactCtrl: public Controller<T>{
            public:
                FullContactCtrl(PlannedTrotTest<T> *, const FloatingBaseModel<T>* );
                virtual ~FullContactCtrl();

                virtual void OneStep(void* _cmd);
                virtual void FirstVisit();
                virtual void LastVisit();
                virtual bool EndOfPhase();

                virtual void CtrlInitialization(const std::string & category_name);
                virtual void SetTestParameter(const std::string & test_file);

            protected:
                PlannedTrotTest<T>* _trot_test;
                DVec<T> _Kp, _Kd;
                DVec<T> _des_jpos; 
                DVec<T> _des_jvel; 
                DVec<T> _des_jacc;

                bool _b_set_height_target;
                T _end_time;
                int _dim_contact;

                Task<T>* _body_pos_task;
                Task<T>* _body_ori_task;

                KinWBC<T>* kin_wbc_;
                ContactSpec<T>* fr_contact_;
                ContactSpec<T>* fl_contact_;
                ContactSpec<T>* hr_contact_;
                ContactSpec<T>* hl_contact_;
                WBLC<T>* wblc_;
                WBLC_ExtraData<T>* wblc_data_;

                T target_body_height_;
                T ini_body_height_;
                Vec3<T> ini_body_pos_;

                void _task_setup();
                void _contact_setup();
                void _compute_torque_wblc(DVec<T> & gamma);

                T _ctrl_start_time;
                ParamHandler* _param_handler;
                StateProvider<T>* _sp;
        };
};

#endif
