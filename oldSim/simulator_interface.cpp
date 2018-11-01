#include "simulator_interface.h"
#include "simulatorlcm/sim_torque_t.hpp"

// main class for simulator
// no longer uses LCM for simulator settings - these are set through the GUI!
// lcm threading also uses fancy new C++ thread stuff
// threads can actually be stopped
// reset functions
using std::fill_n;

SimulatorInterface::SimulatorInterface(cheetah_type robot, float v_bus):
lcm("udpm://239.255.76.67:7667?ttl=1"),
v_bus(v_bus),
lcm_handler(&sim_state)
{

    if(robot == mini_cheetah)
    {
        passive_damping = MINI_PASSIVE_DAMPING;
        dry_friction = MINI_DRY_FRICTION;
    }
    else if(robot == cheetah_3_v1 || robot == cheetah_3_v2)
    {
        passive_damping = C3_PASSIVE_DAMPING;
        dry_friction = C3_DRY_FRICTION;
    }

    terrain_file_location = "../Terrains/Terrain.txt";
    robot_type = robot;

    cout<<"[Cheetah Control] New simulator for robot "<<robot<<"\n";
    e2 = std::mt19937(rd());
    dist_accel = std::uniform_real_distribution<>(-GYRO_DIST,GYRO_DIST);
    dist_gyro = std::uniform_real_distribution<>(-ACC_DIST,ACC_DIST);

    if(lcm.good())
        cout<<"[Cheetah Control] Simulator LCM initialized successfully!\n";
    else
        cout<<"[Cheetah Control] Simulator LCM didn't work.\n";
    //SingleStep_initialize();

    lcm.subscribe("CHEETAH_spi_command",  &sim_lcm_handler::handleSPIMessage, &lcm_handler);
    lcm.subscribe("CHEETAH_ecat_command", &sim_lcm_handler::handleControlMessage, &lcm_handler);
    lcm.subscribe("CHEETAH_loop_counter", &sim_lcm_handler::handleLoopCount, &lcm_handler);

//    // no longer use LCM for simulation settings!
//    //lcm.subscribe("INTERFACE_gui_time_settings", &sim_lcm_handler::handleTimeSettings, &lcm_handler);
//    //lcm.subscribe("INTERFACE_gui_environment_settings", &sim_lcm_handler::handleEnvironmentSettings, &lcm_handler);


    lcm_thread = new std::thread(&SimulatorInterface::handle_lcm,this);


    full_reset_simulation();

    sim_thread = new std::thread(&SimulatorInterface::simulate_robot,this);

    cout<<"[Cheetah Control] Simulator initialization done.\n";
}

// hopefully, when the destructor is called, the threads will be ready to quit
// otherwise the program hangs here forever
SimulatorInterface::~SimulatorInterface()
{
    want_shutdown = true;
    cout<<"[Simulator] Shutdown requested.  Waiting for LCM to stop...\n";
    lcm_thread->join();
    cout<<"done!\n";
    cout<<"[Simulator] Waiting for simulator to step....\n";
    sim_thread->join();
    cout<<"done!\n";
    //delete lcm_thread;
    //delete sim_thread;
}

void SimulatorInterface::set_battery_voltage(float v)
{
    cout<<"[Simulator] Updated DC voltage to "<<v<<"\n";
    v_bus = v;
}

float SimulatorInterface::mini_cheetah_motor_model(float tau_des, float qd, float v, float gr, int leg, int joint)
{

    float* tau_log = nullptr;
    float* v_log = nullptr;

    if(joint == 0)
    {
        tau_log = sim_torque.tau_abad + leg;
        v_log = sim_torque.v_abad + leg;
    }
    else if(joint == 1)
    {
        tau_log = sim_torque.tau_hip + leg;
        v_log   = sim_torque.v_hip + leg;
    }
    else
    {
        tau_log = sim_torque.tau_knee + leg;
        v_log   = sim_torque.v_knee + leg;
    }

    float tau_des_motor = tau_des / gr;
    float i_des = tau_des_motor / KT_MINI_CHEETAH;
    float v_des = i_des * R_MINI_CHEETAH + qd * gr * KT_MINI_CHEETAH;
    float v_act = 0.f;

    if(v_des > v)
        v_act = v;
    else if(v_des < -v)
        v_act = -v;
    else
    {
        *tau_log = tau_des;
        *v_log   = v_des;
        return tau_des;
    }

    *v_log = v_act;
    float tau_act = gr * KT_MINI_CHEETAH * (v_act - KT_MINI_CHEETAH * gr * qd) / R_MINI_CHEETAH;
    *tau_log = tau_act;
    // voltage limited!
    return tau_act;
}

float SimulatorInterface::cheetah_3_motor_model(float tau_des,  float qd, float v, float gr, int leg, int joint)
{

    float* tau_log = nullptr;
    float* v_log = nullptr;

    if(joint == 0)
    {
        tau_log = sim_torque.tau_abad + leg;
        v_log = sim_torque.v_abad + leg;
    }
    else if(joint == 1)
    {
        tau_log = sim_torque.tau_hip + leg;
        v_log   = sim_torque.v_hip + leg;
    }
    else
    {
        tau_log = sim_torque.tau_knee + leg;
        v_log   = sim_torque.v_knee + leg;
    }
    float tau_des_motor = tau_des / gr;
    float i_des = tau_des_motor / KT_CHEETAH_3;
    float v_des = i_des * R_CHEETAH_3 + qd * gr * KT_CHEETAH_3;

    float v_act = 0.f;

    if(v_des > v)
        v_act = v;
    else if(v_des < -v)
        v_act = -v;
    else
    {
        *tau_log = tau_des;
        *v_log   = v_des;
        return tau_des;
    }

    *v_log = v_act;
    float tau_act = gr * KT_CHEETAH_3 * (v_act - KT_CHEETAH_3 * gr * qd) / R_CHEETAH_3;
    *tau_log = tau_act;

    return tau_act;
}

// struct with xfb and q
CheetahState* SimulatorInterface::get_cheetah_state()
{
    return &(sim_state.cheetah_state);
}


// this runs in the lcm thread
// now uses the handleTimeout so it doesn't hang for more than 10 ms
void SimulatorInterface::handle_lcm()
{
    cout<<"[LCM Handler thread] Hello!\n";
    for(;;)
    {
        if(want_shutdown)
            return;
        lcm.handleTimeout(10);
    }
}

// simulation thread
void SimulatorInterface::simulate_robot()
{
    cout<<"[Simulation Thread] Hello!\n";
    for(;;)
    {
        if(want_shutdown)
            return;
        // RESET TIMERS ON FIRST ITERATIONS
        if(first_iteration)
        {
            cout<<"[Cheetah Simulator Thread] First iteration!\n";

            getSysTime( &time_spec_start_sim     );
            getSysTime( &time_spec_last_ectat    );
            getSysTime( &time_spec_last_stdout   );
            getSysTime( &time_spec_last_graphics );
            cout<<"\n\n\n********************SIM START!*****************\n";
        }

        CheetahState cheetah_state_copy;

        // spin on pause, keep copying state until unpaused
        do
        {
            if(want_shutdown)
                return;
            //printf("pause spin.\n");
            sim_state.state_mutex.lock();
            {
                memcpy(&cheetah_state_copy,  &(sim_state.cheetah_state), sizeof(sim_state.cheetah_state) );
                memcpy(&current_time_settings, &(sim_state.time_settings), sizeof(current_time_settings));
                memcpy(gravity, &(sim_state.environment_settings.gravity),sizeof(gravity) );
                //matlab_environment_struct.K = sim_state.environment_settings.ground_k;
                //matlab_environment_struct.D = sim_state.environment_settings.ground_d;
                //matlab_environment_struct.mu = sim_state.environment_settings.mu;
            }
            sim_state.state_mutex.unlock();
            if (current_time_settings.paused)
                usleep(1000);

        } while( current_time_settings.paused > 0 );


        if(timeSince(time_spec_last_stdout) > .016 || first_iteration)
        {
            first_iteration = false;
            static double last_stdout_sim_time = 0;

            sim_state.state_mutex.lock();
            double diff_sim_time  = t_sim- last_stdout_sim_time;
            double diff_real_time = timeSince(time_spec_last_stdout);
            getSysTime(&time_spec_last_stdout);


            ratio_statistic =  diff_sim_time / diff_real_time;
            sim_time_statistic = t_sim;
            //printf("Sim time: %.3lf Sim timestep: %.3lf Real timestep: %.3lf Speed: %.3lf  TI iterations: %d Ethercat iterations: %d\n", t_sim, diff_sim_time, diff_real_time, ratio,
            //        sim_state.boards[0]->data_structure.loop_count_ti, sim_state.boards[0]->data_structure.ethercat_count_ti );

            //for( int i = 0; i < 4 ; i++) {
            //  printf("Leg f %d\t %6.3f \t%6.3f \t%6.3f\n", i,ti_board_data[i].force[0],ti_board_data[i].force[1],ti_board_data[i].force[2]);
            //}

            last_stdout_sim_time = t_sim;
            sim_state.state_mutex.unlock();
        }

        // Wait for controller to catch up
        int waitCntr = 0;
        for(;;)
        {
            //printf("spin...\n");
            if(want_shutdown)
                return;
            //printf("WAITIN FOR SIM INFO\n");
            int64_t control_loop_count_copy;
            sim_state.loop_counter_mutex.lock();
            {
                // this is the number of loops the controller has done
                control_loop_count_copy = sim_state.control_loop_count;
            }
            sim_state.loop_counter_mutex.unlock();

            // If the controller has caught up to the simulator, then proceed to do more simulation
            if( control_loop_count_copy >= loop_counter.loop_count )
            {
                //printf("control loop count: %d\n",control_loop_count_copy);
                break;
            }
            // Else, renotify the controller how far the simulator has progressed (every 100 microseconds)
            waitCntr++;
            if( waitCntr > 10 ) {
                lcm.publish("SIMULATOR_loop_counter", &loop_counter);
                waitCntr = 0;
            }
            usleep(10);
        }

        if(robot_type == cheetah_3_v1 || robot_type == cheetah_3_v2)
        {
            // Do ti board stuff (including adding passive damping/dry friction) if needed
            sim_state.control_mutex.lock();
            {
                //const float side_signs[] = {-1, 1, -1, 1};

                if (t_sim >= next_ti_time)
                {
                    next_ti_time += current_time_settings.time_step_low_level_control;
                    for (int j = 0; j < 4; ++j)
                    {
                        // Copy over state, run TI control, assume perfect torque tracking
                        for (int i = 0; i < 3; ++i)
                        {
                            sim_state.boards[j]->data_structure.q[i]  = cheetah_state_copy.q[3*j+i];
                            sim_state.boards[j]->data_structure.dq[i] = cheetah_state_copy.qd[3*j+i];
                        }
                        sim_state.boards[j]->run_ti_board_iteration(); //side_signs[j], ti_board_command[j], &ti_board_data[j] );
                    }
                }

                // this is a change.
                for(int j = 0; j < NUM_LEGS; j++)
                    memcpy( &(recent_ti_board_data[j]), &(sim_state.boards[j]->data_structure ), sizeof(sim_state.boards[j]->data_structure) );

                int jnum = 0;
                for (int i = 0; i < 4; i+=1) {
                    tau_in[jnum] = cheetah_3_motor_model(sim_state.boards[i]->data_structure.tau_des[0],cheetah_state_copy.qd[jnum],v_bus,GR_CHEETAH_3_HIP_ABAD,i,0)
                            - passive_damping * cheetah_state_copy.qd[jnum]- dry_friction*sign(cheetah_state_copy.qd[jnum]);

                    jnum++;


                    tau_in[jnum] = cheetah_3_motor_model(sim_state.boards[i]->data_structure.tau_des[1], cheetah_state_copy.qd[jnum],v_bus,GR_CHEETAH_3_HIP_ABAD,i,1)
                            - passive_damping * cheetah_state_copy.qd[jnum]- dry_friction*sign(cheetah_state_copy.qd[jnum]);
                    jnum++;

                    tau_in[jnum] = cheetah_3_motor_model(sim_state.boards[i]->data_structure.tau_des[2], cheetah_state_copy.qd[jnum],v_bus,GR_CHEETAH_3_KNEE,i,2)
                            - passive_damping * cheetah_state_copy.qd[jnum]- dry_friction*sign(cheetah_state_copy.qd[jnum]);
                    jnum++;
                }
            }
            sim_state.control_mutex.unlock();
        }
        else
        {
            // Do SPINE board stuff (including adding passive damping/dry friction) if needed
            sim_state.control_mutex.lock();
            if (t_sim >= next_ti_time)
            {
                next_ti_time += current_time_settings.time_step_low_level_control;
                for (int j = 0; j < 4; ++j)
                {
                    // Copy over state, run SPINE control, assume perfect torque tracking
                    sim_state.spi_boards[j]->data->q_abad[j] = cheetah_state_copy.q[3*j];
                    sim_state.spi_boards[j]->data->q_hip[j]  = cheetah_state_copy.q[3*j + 1];
                    sim_state.spi_boards[j]->data->q_knee[j] = cheetah_state_copy.q[3*j + 2];
                    sim_state.spi_boards[j]->data->qd_abad[j] = cheetah_state_copy.qd[3*j];
                    sim_state.spi_boards[j]->data->qd_hip[j]  = cheetah_state_copy.qd[3*j + 1];
                    sim_state.spi_boards[j]->data->qd_knee[j] = cheetah_state_copy.qd[3*j + 2];
                    sim_state.spi_boards[j]->run_spine_board_iteration();
                }
            }

            // copy over most recent spi data to eventually send back over LCM
            memcpy(&recent_spi_data, &sim_state.spi_data, sizeof(cheetahlcm::spi_data_t));

            // loop over all joints and get torques from SPINE board
            int jnum = 0;
            for(int i = 0; i < 4; i++)
            {
                tau_in[jnum] = mini_cheetah_motor_model(sim_state.spi_boards[i]->torque_out[0],cheetah_state_copy.qd[jnum],v_bus,GR_MINI_CHEETAH_HIP_ABAD,i,0)
                        - passive_damping * cheetah_state_copy.qd[jnum]- dry_friction*sign(cheetah_state_copy.qd[jnum]);
                jnum++;

                tau_in[jnum] = mini_cheetah_motor_model(sim_state.spi_boards[i]->torque_out[1],cheetah_state_copy.qd[jnum],v_bus,GR_MINI_CHEETAH_HIP_ABAD,i,1)
                        - passive_damping * cheetah_state_copy.qd[jnum]- dry_friction*sign(cheetah_state_copy.qd[jnum]);
                jnum++;

                tau_in[jnum] = mini_cheetah_motor_model(sim_state.spi_boards[i]->torque_out[2],cheetah_state_copy.qd[jnum],v_bus,GR_MINI_CHEETAH_KNEE,i,2)
                        - passive_damping * cheetah_state_copy.qd[jnum]- dry_friction*sign(cheetah_state_copy.qd[jnum]);
                jnum++;
            }
            sim_state.control_mutex.unlock();
        }

        // integrate!
        int cheetah_number = (robot_type == mini_cheetah?4:3);
        sim_state.state_mutex.lock();
        {
            sim_flt dt_sim = current_time_settings.time_step_integration;

            //SingleStep(sim_state.cheetah_state.xfb, sim_state.cheetah_state.q, sim_state.cheetah_state.qd, sim_state.environment_state.u, tau_in, dt_sim, &matlab_environment_struct, gravity,last_normals, cheetah_number,p, pd, f);
            t_sim+=dt_sim;

            loopCount ++;
            // Publish Simulation State
            if (t_sim >= next_ecat_time)
            {
                // Get the current time, and see if we need to sleep for slowmo
                sim_flt time_desired = current_time_settings.slow_motion_factor * sim_state.time_settings.time_step_high_level_control;
                sim_flt time_actual  = timeSince(time_spec_last_ectat) + 75e-6;
                if (  time_actual  <  time_desired  ) {
                    usleep( (time_desired-time_actual)*1e6  );
                }
                getSysTime(&time_spec_last_ectat);

                next_ecat_time += current_time_settings.time_step_high_level_control;
                memcpy(state_lcm_data.quat    , cheetah_state_copy.xfb     ,sizeof(state_lcm_data.quat));
                memcpy(state_lcm_data.p       , cheetah_state_copy.xfb+4   ,sizeof(state_lcm_data.p));
                memcpy(state_lcm_data.omegab  , cheetah_state_copy.xfb+7   ,sizeof(state_lcm_data.omegab));
                memcpy(state_lcm_data.vb      , cheetah_state_copy.xfb+10  ,sizeof(state_lcm_data.vb));
                memcpy(state_lcm_data.q       , cheetah_state_copy.q       ,sizeof(state_lcm_data.q));
                memcpy(state_lcm_data.qd      , cheetah_state_copy.qd      ,sizeof(state_lcm_data.qd));

                for(int i = 0 ; i < 12 ; i++) {
                    state_lcm_data.qdd[i] = (sim_state.cheetah_state.qd[i]- cheetah_state_copy.qd[i])/dt_sim;
                    state_lcm_data.tau[i] = tau_in[i];// + passive_damping * cheetah_state_copy.qd[i];
                }
                for (int i = 0; i < 3; ++i)
                {
                    state_lcm_data.omegabd[i]= ( sim_state.cheetah_state.xfb[7 +i] - cheetah_state_copy.xfb[7 +i] ) /dt_sim;
                    state_lcm_data.vbd[i]   = ( sim_state.cheetah_state.xfb[10+i] - cheetah_state_copy.xfb[10+i] ) /dt_sim;
                }
                for(int j = 0 ; j < 4 ; j++) {
                    for (int k = 0 ; k < 3 ; k++) {
                        state_lcm_data.f_foot[j][k]   = f[ (9+2*j)*3 + k];
                        state_lcm_data.fvec[3*j + k]  = f[ (9+2*j)*3 + k];
                        state_lcm_data.p_footvec[3*j + k] = p[(9+2*j)*3 + k];
                    }
                }

                // Convert orientation information into state_lcm_struct data
                SimulatorMath::QuatToRpy(state_lcm_data.quat, state_lcm_data.rpy);
                SimulatorMath::OmegabToRpyRates(state_lcm_data.omegab, state_lcm_data.rpy, state_lcm_data.rpyd );
                SimulatorMath::QuatToR(state_lcm_data.quat, state_lcm_data.R );

                for(int i = 0 ; i <3 ; i++) {
                    for(int j = 0 ; j< 3; j++) {
                        state_lcm_data.Rvec[3*j+i] = state_lcm_data.R[i][j];
                    }
                }

                float agb[3], accelerometer[3], RT[3][3];
                for(int i = 0 ; i < 3 ; i++) {
                    for( int j = 0 ; j < 3 ; j++) {
                        RT[i][j] = state_lcm_data.R[j][i];
                    }
                }

                // Compute cartesian acceleration from body acceleration data and velocity data
                // Note that vb is the body velocity in body coordinates
                //      as a result, one must be careful to remove omega x v effects on the body velocity
                // TODO: Write up some math on these lines in the control documentation
                SimulatorMath::cross_product(state_lcm_data.omegab,state_lcm_data.vb, accelerometer);
                SimulatorMath::apply_cartesian_tensor(RT, gravity, agb );
                for (int i = 0; i < 3; ++i) {
                    agb[i]*=-1;
                }
                SimulatorMath::add_cartesian_vector(accelerometer, agb);
                SimulatorMath::add_cartesian_vector( accelerometer, state_lcm_data.vbd );
                for (int i = 0; i < 3; ++i)
                {
                    // Add articifical noise
                    imu_data.gyro[i]          = state_lcm_data.omegab[i] + dist_gyro(e2) ;
                    imu_data.accelerometer[i] = accelerometer[i] / 9.81  + dist_accel(e2) ;

//                    vectornav_data.gyro[i] = imu_data.gyro[i];
//                    vectornav_data.acc[i]  = imu_data.accelerometer[i];
                }

                // transformed gyro/acc in the IMU's coordinate system
                // robot to imu is [0 0 1; 0 -1 0; 1 0 0]
                // its symmetric, so I can't get it backward.
                vectornav_data.gyro[0] = state_lcm_data.omegab[0] + dist_gyro(e2);
                vectornav_data.gyro[1] = state_lcm_data.omegab[1] + dist_gyro(e2);
                vectornav_data.gyro[2] = state_lcm_data.omegab[2] + dist_gyro(e2);

                vectornav_data.acc[0]  = accelerometer[0] + dist_accel(e2);
                vectornav_data.acc[1]  = accelerometer[1] + dist_accel(e2);
                vectornav_data.acc[2]  = accelerometer[2] + dist_accel(e2);

                // vectornav orders the quaternion differently
                vectornav_data.quat[0] = state_lcm_data.quat[1];
                vectornav_data.quat[1] = state_lcm_data.quat[2];
                vectornav_data.quat[2] = state_lcm_data.quat[3];
                vectornav_data.quat[3] = state_lcm_data.quat[0];

                // TODO: add noise to the simulated quaternion
                // or better yet, simulate the vectornav's algorithm
                // for now, we assume the vectornav's orientation
                // estimate is perfect in simulation.

                // also need to rotate quat.
//                sim_flt temp_quat[4];
//                sim_flt rot_quat[4] = {0.f, -0.70710678f, 0.f, -0.70710678f};
//                for(int i = 0; i < 4; i++)
//                    temp_quat[i] = state_lcm_data.quat[i];

//                SimulatorMath::quatProduct(temp_quat, rot_quat, vectornav_data.quat);

                loop_counter.loop_count ++;
                lcm.publish("SIMULATOR_loop_counter", &loop_counter);
                if(robot_type == mini_cheetah)
                    lcm.publish("SIMULATOR_vectornav_data", &vectornav_data);
                else
                    lcm.publish("SIMULATOR_imu_data", &imu_data);
                lcm.publish("SIMULATOR_full_state", &state_lcm_data);

                // Copy and publish mock ethercat data
                for (int i = 0; i < 4; ++i)
                {
                    int side_sign = pow(-1,(i+1));
                    ecat_data.x[i] = recent_ti_board_data[i].position[0];
                    ecat_data.y[i] = recent_ti_board_data[i].position[1];
                    ecat_data.z[i] = recent_ti_board_data[i].position[2];

                    ecat_data.dx[i] = recent_ti_board_data[i].velocity[0];
                    ecat_data.dy[i] = recent_ti_board_data[i].velocity[1];
                    ecat_data.dz[i] = recent_ti_board_data[i].velocity[2];

                    ecat_data.fx[i] = recent_ti_board_data[i].force[0];
                    ecat_data.fy[i] = recent_ti_board_data[i].force[1];
                    ecat_data.fz[i] = recent_ti_board_data[i].force[2];

                    ecat_data.q_abad[i] = recent_ti_board_data[i].q[0];
                    ecat_data.q_hip[i]  = recent_ti_board_data[i].q[1];
                    ecat_data.q_knee[i] = recent_ti_board_data[i].q[2];

                    ecat_data.dq_abad[i] = recent_ti_board_data[i].dq[0];
                    ecat_data.dq_hip[i]  = recent_ti_board_data[i].dq[1];
                    ecat_data.dq_knee[i] = recent_ti_board_data[i].dq[2];

                    ecat_data.tau_abad[i] = side_sign*recent_ti_board_data[i].tau[0];
                    ecat_data.tau_hip[i]  = side_sign*recent_ti_board_data[i].tau[1];
                    ecat_data.tau_knee[i] = side_sign*recent_ti_board_data[i].tau[2];

                    ecat_data.tau_des_abad[i] = side_sign*recent_ti_board_data[i].tau_des[0];
                    ecat_data.tau_des_hip[i]  = side_sign*recent_ti_board_data[i].tau_des[1];
                    ecat_data.tau_des_knee[i] = side_sign*recent_ti_board_data[i].tau_des[2];

                    ecat_data.loop_count_ti[i] = recent_ti_board_data[i].loop_count_ti;
                    ecat_data.ethercat_count_ti[i] = recent_ti_board_data[i].ethercat_count_ti;
                    ecat_data.microtime_ti[i] = recent_ti_board_data[i].microtime_ti;
                }
                if(robot_type == cheetah_3_v1 || robot_type == cheetah_3_v2)
                    lcm.publish("SIMULATOR_ecat_data", &ecat_data);
                else
                {
                    recent_spi_data.spi_driver_status = (spi_id_counter++ << 16) + 0xba;

                    lcm.publish("SIMULATOR_spi_data", &recent_spi_data);
                }

                lcm.publish("SIMULATOR_sim_torque", &sim_torque);
            }

            // Publish SimGraphics State
            if( timeSince(time_spec_last_graphics) >= 1./ sim_state.time_settings.graphics_frames_per_second ) {
                getSysTime(&time_spec_last_graphics);

                memcpy(graphics_lcm_data.xfb  , sim_state.cheetah_state.xfb  , sizeof(graphics_lcm_data.xfb));
                memcpy(graphics_lcm_data.q    , sim_state.cheetah_state.q    , sizeof(graphics_lcm_data.q));
                graphics_lcm_data.sim_time = t_sim;

                // Copy foot data
                for(int j = 0 ; j < 4 ; j++) {
                    for (int k = 0 ; k < 3 ; k++) {
                        graphics_lcm_data.f_foot[k][j]  = f[ (9+2*j)*3 + k];
                        graphics_lcm_data.p_foot[k][j]  = p[ (9+2*j)*3 + k];
                    }
                }
                lcm.publish("SIMULATOR_sim_graphics", &graphics_lcm_data);
            }
        }
        sim_state.state_mutex.unlock();


    }

}

sim_flt SimulatorInterface::sign(sim_flt x)
{
    return x<0.f ? -1.f : 1.f;
}

// resets only the member flaots/arrays of this class
// doesn't touch the state
void SimulatorInterface::reset_simulator_variables()
{
    fill_n(tau_in, 12, 0);
    fill_n(p, NUM_CONTACT_PTS*3, 0);
    fill_n(pd,NUM_CONTACT_PTS*3, 0);
    fill_n(f, NUM_CONTACT_PTS*3, 0);
    fill_n(last_normals, NUM_CONTACT_PTS*3,0);
    for (int i = 0; i < NUM_CONTACT_PTS; ++i)
        last_normals[3*i+2] = 1;

    for (int i = 0; i < 3; i++)
        gravity[i] = 0;

    gravity[2] = -9.81;

    next_ecat_time = 0;
    next_ti_time = 0;
    t_sim = 0;
    loopCount = 0;
    loop_counter.loop_count = 0;
    first_iteration = true;
}

// resets state and variables
void SimulatorInterface::fast_reset_simulation()
{
    cout<<"[Cheetah Control] Simulation Resetting (fast)...\n";
    sim_state.reset_state();
    reset_simulator_variables();
}

// resets state, variables, reloads terrain, reloads robot lengths
void SimulatorInterface::full_reset_simulation()
{
    cout<<"[Cheetah Control] Simulation Resetting (full)...\n";
    InitGlobalTerrainInfoFromFile(terrain_file_location.c_str());
    cout<<"[SIM] Terrain reloaded\n";

    if(robot_type == mini_cheetah)
        sim_state.set_link_lengths(MINI_L1, MINI_L2, MINI_L3);
    else
        sim_state.set_link_lengths(C3_L1, C3_L2, C3_L3);

    sim_state.reset_state();
    reset_simulator_variables();
}

sim_flt SimulatorInterface::timeDiff(const timespec &t1, const timespec &t2)
{
    return ((double) t2.tv_sec - t1.tv_sec) + (1.0e-9*((double) t2.tv_nsec - t1.tv_nsec));
}

sim_flt SimulatorInterface::timeSince(const timespec &t1)
{
    timespec t2;
    getSysTime(&t2);
    return timeDiff(t1,t2);
}

void SimulatorInterface::getSysTime(timespec *ts)
{
#if defined(_POSIX_TIMERS) && !defined(__APPLE__)
    if (clock_gettime(CLOCK_REALTIME, ts) != 0)
    {
        throw ts;
    }
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    ts->tv_sec = tv.tv_sec;
    ts->tv_nsec = 1000*tv.tv_usec;
#endif
}
