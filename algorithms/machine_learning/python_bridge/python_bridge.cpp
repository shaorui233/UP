#include <iostream>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <cpp/simulation_to_python.hpp>
#include <cpp/python_to_simulation.hpp>
#include <unistd.h>
#include <Utilities/Timer.h>
#include <Dynamics/Quadruped.h>

simulation_to_python output;

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const simulation_to_python * msg)
        {
            output = *msg;
//            printf("Received message on channel \"%s\":\n", chan.c_str());
        }
};

int step(double* in_jpos, double* in_jvel, double* out_config, double* out_config_vel, bool reset)
{
//    printf("Python Bridge\n");
    lcm::LCM lcm_subscribe;
    lcm::LCM lcm_publish;

    if(!lcm_subscribe.good())
        return 1;

    python_to_simulation input;
    Handler handlerObject;

    Timer time;

    lcm_subscribe.subscribe("simulation_to_python", &Handler::handleMessage, &handlerObject);
    input.reset_call = reset; //false;
    
    for(size_t i(0); i<cheetah::num_act_joint; ++i){
        input.jpos_cmd[i] = in_jpos[i];
        input.jvel_cmd[i] = in_jvel[i];
    }

    lcm_publish.publish("python_to_simulation", &input);
    lcm_subscribe.handle();

    for(size_t i(0); i<cheetah::dim_config; ++i){
        out_config[i] = output.config[i];
        out_config_vel[i] = output.config_vel[i];
    }
//    printf("%f ms\n", time.getMs());
    return 0;
}
