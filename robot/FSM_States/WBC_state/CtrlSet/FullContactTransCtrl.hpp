#ifndef FULL_CONTACT_TRANSITION_CONFIGURATION_CTRL
#define FULL_CONTACT_TRANSITION_CONFIGURATION_CTRL

#include <WBC_state/Controller.hpp>

template <typename T> class ContactSpec;
template <typename T> class WBLC;
template <typename T> class WBLC_ExtraData;
template <typename T> class KinWBC;
template <typename T> class Cheetah_StateProvider;


template <typename T>
class FullContactTransCtrl: public Controller<T>{
    public:
        FullContactTransCtrl(const FloatingBaseModel<T>* );
        virtual ~FullContactTransCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setStanceTime(T stance_time){ end_time_ = stance_time; }
        void setStanceHeight(T height) {
            target_body_height_ = height;
            b_set_height_target_ = true;
        }

    protected:
               Task<T>* body_pos_task_;
        Task<T>* body_ori_task_;

        ContactSpec<T>* fr_contact_;
        ContactSpec<T>* fl_contact_;
        ContactSpec<T>* hr_contact_;
        ContactSpec<T>* hl_contact_;

        KinWBC<T>* kin_wbc_;
        WBLC<T>* wblc_;
        WBLC_ExtraData<T>* wblc_data_;

        DVec<T> base_pos_ini_;
        Vec3<T> ini_base_pos_;

        DVec<T> Kp_;
        DVec<T> Kd_;
        
        DVec<T> ini_jpos_;
        DVec<T> des_jpos_;
        DVec<T> des_jvel_;
        DVec<T> des_jacc_;

        bool b_set_height_target_;
        T end_time_;
        T target_body_height_;
        T ini_body_height_;
        Vec3<T> ini_body_pos_;
        
        T max_rf_z_;
        T min_rf_z_;
        int dim_contact_;
        T ctrl_start_time_;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wblc(DVec<T> & gamma);

        Cheetah_StateProvider<T>* sp_;
};

#endif
