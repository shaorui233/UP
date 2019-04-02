#ifndef PLANNER_H
#define PLANNER_H

#include <cppTypes.h>
#include "Path.hpp"
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/StateProvider.hpp>

template <typename T>
class Planner{
    public:
        Planner(int first_stance_foot);
        ~Planner();

        void UpdateUserInput(T* dir, T* ori);
        void updateStepIdx(const T& curr_time);
        void getBodyConfig(const T & time, 
                Vec3<T>  & body_pos, Vec3<T>  & body_vel, Vec3<T> & body_acc, 
                Vec3<T>  & body_ori, Vec3<T>  & body_ang_vel );

        void getNextFootLocation(int swing_foot, 
                Vec3<T> & front_foot, Vec3<T> & hind_foot);

        void SetParameter(const std::string & config_file);
        void updateExtraData(Cheetah_Extra_Data<T>* ext_data);

        bool stop(const T & curr_time);

        Path<T> _engaged_path;
        Path<T> _replanned_path;

        int _path_flag;
        T _curr_path_st_time;
        T _body_height;
        T _min_trot_initiating_step_size = 0.01;
        int _step_idx; // -1: before start, -2: end of motion
        int _first_stance_foot;

        Vec3<T> _curr_global_frame_loc;
        StateProvider<T>* _sp;
};

#endif
