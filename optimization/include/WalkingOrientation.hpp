#ifndef WALKING_ORIENTATION
#define WALKING_ORIENTATION

#include <Utilities/BSplineBasic.h>
#include <height_map/HeightMap.hpp>
#include <vector>

class WalkingOrientation{
    public:
        WalkingOrientation();
        ~WalkingOrientation();

        virtual bool SolveOptimization() = 0;
        HeightMap* _hmap;

        // Num optimization variable:
        // 4 x 46 (184) step location
        // 3 x 46 (138) body trajectory ctrl points
        // 2 x 46 (138) body orientation (pitch & yaw) trajectory ctrl points
        constexpr static int _nStep = 46;
        constexpr static double _onestep_duration = 0.2;
        constexpr static double _half_body_length = 0.25;
        constexpr static double _half_body_width = 0.17;
        // For easy computation
        constexpr static double _tot_time = (_nStep - 1)*_onestep_duration;
        constexpr static int nMiddle_pt = (_nStep - 2); // _nStep - initial - final
        constexpr static int idx_offset = 4*_nStep;

        double _ini_front_foot_loc[2];
        double _ini_hind_foot_loc[2];

        double _fin_fr_loc[2];
        double _fin_hl_loc[2];
        double _fin_fl_loc[2];
        double _fin_hr_loc[2];

        double _ini_body_pos[3];
        double _fin_body_pos[3];

        double _ini_body_ori[2];
        double _fin_body_ori[2];

        double _min_leg_length;
        double _max_leg_length;

        double _des_leg_length;

        std::string _folder_name;
        int _count;
        int _intermittent_step_size;

        // Utility functions
        void _SetInitialGuess(std::vector<double> & x);
        static void nice_print_result(const std::vector<double> & x);

        static void buildFootStepLocation(const std::vector<double> & x, 
                std::vector<std::vector<double>> & foot_loc_list, const HeightMap* hmap);

        static void SaveOptimizationResult(const std::string& folder, 
                const int & iter, const double & opt_cost, 
                const std::vector<double> & x_best, const WalkingOrientation* tester);

        static void buildSpline(const std::vector<double> & x, 
                BS_Basic<double, 3, 3, nMiddle_pt, 2, 2> & pos_spline,
                BS_Basic<double, 2, 3, nMiddle_pt, 2, 2> & ori_spline);

        // Cost Computation
        //static double LegLengthCost()
        // Constraint
        static void InitialFinalConstraint(
                unsigned m, double * result, unsigned n, const double *x, 
                double * grad, void*data);

        static void KinematicsConstraint(unsigned m, double* result, unsigned n, 
                const double *x, double *grad, void* data);
};

#endif
