#ifndef PATH_H
#define PATH_H

#include <cppTypes.h>
#include <Utilities/BSplineBasic.h>
#include <ParamHandler.hpp>

namespace stance_foot{
    // Must be same value with different sign
    constexpr int FRHL = -1;
    constexpr int FLHR = 1;
};
namespace path_flag{
    constexpr int swap_call = 0;
    constexpr int end_path = 1;
    constexpr int step = 2;
};

template <typename T>
class Path{
    public:
        Path();
        ~Path();

        void printPathInfo();
        void initialization(T* lin_ini, T* ang_ini, int stance_foot);
        void updatePath(T* dir, T* ori, T* ini_lin, T* ini_ori, int ini_stance_foot);
        int checkIdx(const int & step_idx);
        bool checkEnd(const size_t & idx);

        void getSwingFootLoc(const size_t & idx, Vec3<T> & front_foot, Vec3<T> & hind_foot);
        void getPathInfo(const int & idx, const T & time, 
                T* body_lin, T* body_ori, int & stance_foot);

        constexpr static int nMiddle = 5;

        // Stance foot list
        vectorAligned<Vec3<T> > _front_footloc_list; // For trot (front)
        vectorAligned<Vec3<T> > _hind_footloc_list; // For trot (hind)
        std::vector<int> _stance_foot_list; // For trot (front, hind)

        T** mid_ori; 
        T** mid_lin; 
        BS_Basic<T, 3, 3, nMiddle, 2, 2> _lin_spline;
        BS_Basic<T, 3, 3, nMiddle, 2, 2> _ori_spline;

        size_t _end_idx;
        int _path_idx;

        T _step_size[2];
        T _step_size_ori[3];

        T _fin_lin[9]; // Pos, vel, acc
        T _fin_ori[9]; // Pos, vel, acc

        bool _b_used;
        // config file based setup 
        void setParameters(ParamHandler* );

        Vec3<T> _fr_shoulder;
        Vec3<T> _fl_shoulder;
        Vec3<T> _hr_shoulder;
        Vec3<T> _hl_shoulder;
        std::vector<T> _step_size_min;
        std::vector<T> _step_size_max;

        std::vector<T> _step_size_ori_min;
        std::vector<T> _step_size_ori_max;

        Vec3<T> _path_start_loc;

        T _step_time;
        T _tot_time;

        inline const Path & operator = (const Path& p){
            this->_end_idx = p._end_idx;
            this->_path_idx = p._path_idx;

            this->_step_size[0] = p._step_size[0];
            this->_step_size[1] = p._step_size[1];
            for(size_t i(0); i<9; ++i){
                this->_fin_lin[i] = p._fin_lin[i];
                this->_fin_ori[i] = p._fin_ori[i];
            }
            this->_lin_spline = p._lin_spline;
            this->_ori_spline = p._ori_spline;

            // Foot
            this->_front_footloc_list = p._front_footloc_list;
            this->_hind_footloc_list = p._hind_footloc_list;
            this->_stance_foot_list = p._stance_foot_list;

            return *this;
        }
        void computeFootLoc(const Mat3<T> & rot, const Vec3<T> & shoulder, 
                const T & step_time, 
                const Vec3<T> & body_pos, const Vec3<T> & body_vel, 
                const Vec3<T> & body_ang_vel, Vec3<T> & foot_loc);
     protected:
        void _updateStepLocation(const int & ini_stance_foot);
       T _bound(const T & value, const T & min, const T & max);
};
#endif
