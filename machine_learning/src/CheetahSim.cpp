#include <iostream>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <cpp/simulation_to_python.hpp>
#include <cpp/python_to_simulation.hpp>
#include <Dynamics/Quadruped.h>
#include <Utilities/Utilities_print.h>
#include <WBC_state/Cheetah_DynaCtrl_Definition.h>

class Handler 
{
    public:
        Handler():
            _jpos_cmd(cheetah::num_act_joint),
            _jvel_cmd(cheetah::num_act_joint),
            _state(cheetah::dim_config *2)

    { _count = 0; }
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const python_to_simulation* msg) {
            if(msg->reset_call){
                printf("resect call\n");
                pretty_print(msg->state, "state", cheetah::dim_config * 2);

            }

            //if(_count %10){
            //printf("Received message on channel \"%s\":\n", chan.c_str());
            pretty_print(msg->jpos_cmd, "jpos_cmd", cheetah::num_act_joint);
            pretty_print(msg->jvel_cmd, "jvel_cmd", cheetah::num_act_joint);
            printf("\n");
            //}

            for(size_t i(0); i<cheetah::num_act_joint; ++i){
                _jpos_cmd[i] = msg->jpos_cmd[i];
                _jvel_cmd[i] = msg->jvel_cmd[i];
            }
            for(size_t i(0); i<cheetah::dim_config; ++i){
                _state[i] = msg->state[i];
            }
            ++_count;
        }
    private:
        DVec<double> _jpos_cmd;
        DVec<double> _jvel_cmd;
        DVec<double> _state;

        size_t _count;
};

int main(int argc, char** argv)
{
    lcm::LCM lcm_subscribe;
    lcm::LCM lcm_publish;

    if(!lcm_subscribe.good()) return 1;
    if(!lcm_publish.good()) return 1;

    simulation_to_python output;
    Handler handlerObject;
    lcm_subscribe.subscribe("python_to_simulation", &Handler::handleMessage, &handlerObject);
    while(true){
        lcm_subscribe.handle();
        printf("doing some simulation...\n");
        lcm_publish.publish("simulation_to_python", &output);
    }
    return 0;
}
