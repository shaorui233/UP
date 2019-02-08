#ifndef JPOS_CTRL_VALKYRIE
#define JPOS_CTRL_VALKYRIE

#include <WBC_state/Controller.hpp>
#include <WBC_state/Cheetah_DynaCtrl_Definition.h>

template <typename T> class ContactSpec;
template <typename T> class WBLC;
template <typename T> class WBLC_ExtraData;
template <typename T> class KinWBC;
template <typename T> class Cheetah_StateProvider;

template <typename T>
class JPosCtrl: public Controller<T>{
    public:
        JPosCtrl(const FloatingBaseModel<T>* );
        virtual ~JPosCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setMovingTime(T time) { end_time_ = time; }
        void setTargetPos(const std::vector<T> & jpos_target){ 
            for(size_t i(0); i<cheetah::num_act_joint; ++i)
                _jpos_target[i] = jpos_target[i];

            b_set_target_ = true;
        }

    protected:
        DVec<T> Kp_, Kd_;

        DVec<T> des_jpos_; 
        DVec<T> des_jvel_; 
        DVec<T> des_jacc_;

        DVec<T> jpos_ini_;
        bool b_set_target_;
        T end_time_;
        int dim_contact_;
        T ctrl_start_time_;
        
        DVec<T> _jpos_target;


        ContactSpec<T>* contact_;
        WBLC<T>* wblc_;
        WBLC_ExtraData<T>* wblc_data_;


        void _task_setup();
        void _contact_setup();
        void _compute_torque_wblc(DVec<T> & gamma);

        Cheetah_StateProvider<T>* sp_;
};

#endif
