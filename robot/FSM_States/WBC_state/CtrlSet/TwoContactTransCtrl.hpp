#ifndef TWO_CONTACT_TRANSITION_CHEETAH
#define TWO_CONTACT_TRANSITION_CHEETAH

#include <WBC_state/Controller.hpp>

template <typename T> class ContactSpec;
template <typename T> class WBLC;
template <typename T> class WBLC_ExtraData;
template <typename T> class KinWBC;
template <typename T> class Cheetah_StateProvider;

template <typename T>
class TwoContactTransCtrl: public Controller<T>{
    public:
        TwoContactTransCtrl(const FloatingBaseModel<T>* robot, 
                size_t cp1, size_t cp2, int transit_dir);
        virtual ~TwoContactTransCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);
        virtual void SetTestParameter(const std::string & test_file);

    protected:
        void _SetContact(const size_t & cp_idx, const T & upper_lim, 
                const T & rf_weight, const T & rf_weight_z, const T & foot_weight);

        size_t _cp1, _cp2;
        int _transit_dir; // 1: swing start, -1: swing end
 
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
        T _body_height_cmd;
        T ini_body_height_;
        Vec3<T> ini_body_pos_;
        
        T max_rf_z_;
        T min_rf_z_;
        int dim_contact_;
        T ctrl_start_time_;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wblc(DVec<T> & gamma);

        Cheetah_StateProvider<T>* _sp;
        std::string _test_file_name;
};

#endif
