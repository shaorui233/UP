#ifndef DYNAMICS_SIMULATION_MACHINE_LEARNING
#define DYNAMICS_SIMULATION_MACHINE_LEARNING

#include <string>
#include <cppTypes.h>
#include <SimGraphics3D.hpp>

#include <Dynamics/MiniCheetah.h>
#include <Dynamics/Cheetah3.h>
#include <Dynamics/DynamicsSimulator.h>
#include <thread>
#include <SimUtilities/SpineBoard.h>
#include <ControlParameters/RobotParameters.h>

#include <cpp/simulation_to_python.hpp>
#include <cpp/python_to_simulation.hpp>
#include <lcm/lcm-cpp.hpp>

class Handler {
    public:
        Handler():
            _jpos_cmd(cheetah::num_act_joint),
            _jvel_cmd(cheetah::num_act_joint),
            _state(cheetah::dim_config *2)    { _count = 0; }
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const python_to_simulation* msg) {
            //if(msg->reset_call){
                //printf("resect call\n");
                //pretty_print(msg->state, "state", cheetah::dim_config * 2);
            //}

            //pretty_print(msg->jpos_cmd, "jpos_cmd", cheetah::num_act_joint);
            //pretty_print(msg->jvel_cmd, "jvel_cmd", cheetah::num_act_joint);
            //printf("\n");

            _reset_call = msg->reset_call;
            for(size_t i(0); i<cheetah::num_act_joint; ++i){
                _jpos_cmd[i] = msg->jpos_cmd[i];
                _jvel_cmd[i] = msg->jvel_cmd[i];
            }
            for(size_t i(0); i<cheetah::dim_config; ++i){
                _state[i] = msg->state[i];
            }
            ++_count;
        }
        
        bool _reset_call;
        DVec<double> _jpos_cmd;
        DVec<double> _jvel_cmd;
        DVec<double> _state;

        size_t _count;
};

class DynSimulation{
    public:

        DynSimulation(SimGraphics3D* window);
        ~DynSimulation();

        void reset(DVec<double> & state);
        void step(const DVec<double> & jpos_cmd, const DVec<double> & jvel_cmd, 
                DVec<double> & nx_full_config, DVec<double> & nx_full_vel);

        void setRobotState(FBModelState<double>& state) {
            _simulator->setState(state);
        }

        void run();
        std::thread * lcm_loop_thread;
    private:
        SpineBoard _spineBoards[4];

        double _dt = 0.001;
        double _dtLowLevelControl = 0.0002;
        double _dtHighLevelControl = 0.001;

        double _mu = 0.7;
        double _resti = 0.0;
        size_t _robotID;

        DVec<double> _tau;
        DVec<double> _torque_cmd;
        void _updateGraphics();
        void _SetupParam();
        void _tau_update();

        std::thread _simThread;
        SimGraphics3D *_window = nullptr;
        std::vector<ActuatorModel<double>> _actuatorModels;
        RobotControlParameters _robotParams;

        DynamicsSimulator<double>* _simulator = nullptr;
        FloatingBaseModel<double> _model;
        Quadruped<double> _quadruped;
        DVec<double> _state;
        DVec<double> _full_vel;
        bool _contact[4];

        // LCM setup
        lcm::LCM lcm_subscribe;
        lcm::LCM lcm_publish;
        
        simulation_to_python _output;
        Handler* handler;
        void LCM_Loop();
        void _updateOutput(
                const DVec<double> & full_config, const DVec<double> & full_vel, 
                simulation_to_python & output);
};

#endif
