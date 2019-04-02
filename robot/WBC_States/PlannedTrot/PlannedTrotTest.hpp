#ifndef PLANNED_TROT_TEST_Cheetah
#define PLANNED_TROT_TEST_Cheetah

#include <WBC_States/Test.hpp>
#include <Dynamics/Quadruped.h>

template <typename T> class StateProvider;
template <typename T> class Planner;

namespace PlannedTrotPhase{
    constexpr int lift_up = 0;
    constexpr int full_contact_1 = 1;
    constexpr int frhl_swing_start_trans = 2;
    constexpr int frhl_swing = 3;
    constexpr int frhl_swing_end_trans = 4;
    constexpr int full_contact_2 = 5;
    constexpr int flhr_swing_start_trans = 6;
    constexpr int flhr_swing = 7;
    constexpr int flhr_swing_end_trans = 8;
    constexpr int NUM_TROT_PHASE = 9;
};


template <typename T>
class PlannedTrotTest: public Test<T>{
    public:
        PlannedTrotTest(FloatingBaseModel<T>* , const RobotType& );
        virtual ~PlannedTrotTest();

        DVec<T> _jpos_des_pre;
        Planner<T>* _planner;

        Vec3<T> _body_pos;
        Vec3<T> _body_vel;
        Vec3<T> _body_acc;

        Vec3<T> _body_ori_rpy;
        // This does not have any frame meaning. just derivation of rpy
        Vec3<T> _body_ang_vel; 

        Vec3<T> _front_foot_loc;
        Vec3<T> _hind_foot_loc;

        std::string _folder_name;
    protected:
        T _step_time;
        T _tot_time;

        virtual void _UpdateTestOneStep();
        virtual void _TestInitialization();
        virtual int _NextPhase(const int & phase);
        virtual void _UpdateExtraData(Cheetah_Extra_Data<T> * ext_data);

        void _SettingParameter();

        Controller<T>* body_up_ctrl_;
        Controller<T>* body_ctrl_;

        // Front Right and Hind Left leg swing
        Controller<T>* frhl_swing_start_trans_ctrl_;
        Controller<T>* frhl_swing_ctrl_;
        Controller<T>* frhl_swing_end_trans_ctrl_;

        // Front Left and Hind Right leg swing
        Controller<T>* flhr_swing_start_trans_ctrl_;
        Controller<T>* flhr_swing_ctrl_;
        Controller<T>* flhr_swing_end_trans_ctrl_;

        StateProvider<T>* _sp;
};

#endif
