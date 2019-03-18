#ifndef PLANNER_H
#define PLANNER_H

#include <cppTypes.h>
#include <Utilities/BSplineBasic.h>

template <typename T>
class Path{
    public:
        Path();
        ~Path();

        constexpr static int nMiddle = 5;

        void append(size_t insert_idx);
        vectorAligned<Vec6<T> > _step_loc_list; // For trot (front, hind)
        BS_Basic<T, 3, 3, nMiddle, 2, 2> _pos_spline;
        BS_Basic<T, 3, 3, nMiddle, 2, 2> _ori_spline;

        size_t _curr_step_idx;

        size_t _end_idx;
};
template <typename T>
class Planner{
    public:
        Planner();
        ~Planner();

        void UpdateUserInput(T* dir, T* ori);
        void getBodyPosture(const T & time, DVec<T>  & body_pos, DVec<T>  & body_ori );
        void SetParameter(const std::string & config_file);

        bool stop();

        T _step_size[2];
        std::vector<T> _step_size_min;
        std::vector<T> _step_size_max;
        
        T _step_time;
        T _tot_time;
        
        T _ini_pos[9]; // Pos, vel, acc
        T _ini_ori[9]; // Pos, vel, acc

        T _fin_pos[9]; // Pos, vel, acc
        T _fin_ori[9]; // Pos, vel, acc

        DVec<T> _body_pos;
        DVec<T> _body_ori;

        constexpr static int nMiddle = 5;
        BS_Basic<T, 3, 3, nMiddle, 2, 2> _pos_spline;
        BS_Basic<T, 3, 3, nMiddle, 2, 2> _ori_spline;

        bool _update_call;
        Path<T> _engaged_path;
        Path<T> _replanned_path;
        

        BS_Basic<T, 3, 3, nMiddle, 2, 2> _user_guide_body_path;
        T _bound(const T & value, const T & min, const T & max);

};
#endif
