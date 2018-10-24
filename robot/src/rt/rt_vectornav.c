#include <stdio.h>
#include <inttypes.h>
#include <pthread.h>


#include <rt/rt_vectornav.h>
#include "vn/sensors.h"


#include <rt/rt_lcm.h>
#include <cheetahlcm_vectornav_data_t.h>

// vectornav gets its own LCM
// so it outputs messages that are timestamped
// with the actual time the sensor sends a message
static lcm_t* lcm_vectornav;
volatile cheetahlcm_vectornav_data_t vectornav_data;
pthread_mutex_t vectornav_data_mutex;

#define K_MINI_CHEETAH_VECTOR_NAV_SERIAL "/dev/ttyS0"


//#define PRINT_VECTORNAV_DEBUG

// if uncommented, there's an LCM publish in the vectornav
// message handler to log the data over lcm as
// "SENSOR_vectornav_data"
// this is redundant - this same data is logged at 1 kHz from the state estimator
// but might be useful for debugging.
// it also might make the handler function take too long to return and break things...
#define LOG_VECTORNAV

int processErrorReceived(char* errorMessage, VnError errorCode);
void vectornav_handler(void* userData, VnUartPacket *packet, size_t running_index);

// receive LCM message for vectornav
static void lcm_handler_vectornav(const lcm_recv_buf_t* rbuf, const char* chan, const cheetahlcm_vectornav_data_t* msg, void* n)
{
    pthread_mutex_lock(&vectornav_data_mutex);
    memcpy((void*)&vectornav_data, msg, sizeof(vectornav_data));
    pthread_mutex_unlock(&vectornav_data_mutex);

    // for debugging
    //printf("got vn lcm\n");
}

static void* read_lcm_vectornav(void* a)
{
    printf("[RT Vectornav] Simulator vectornav read thread started\n");
    while(1)
        lcm_handle(lcm_vectornav);
}

typedef struct
{
    VnSensor vs;
    BinaryOutputRegister bor;
} vn_sensor;

vn_sensor vn;

void init_vectornav(int simulator)
{

    lcm_vectornav = lcm_create("udpm://239.255.76.67:7667?ttl=255");

    if(!lcm_vectornav)
    {
        printf("[ERROR: RT Vectornav] failed to init lcm for sim receive.\n");
        return;
    }

    if(simulator)
    {
        printf("[RT Vectornav] Initializing for simulation...\n");

        if(pthread_mutex_init(&vectornav_data_mutex, NULL) != 0)
        {
            printf("[ERROR: RT Vectornav] failed to init mutex.\n");
            return;
        }



        cheetahlcm_vectornav_data_t_subscribe(lcm_vectornav,
                           "SIMULATOR_vectornav_data", &lcm_handler_vectornav, NULL);

        pthread_t vectornav_read_thread;
        int thread_rc = pthread_create(&vectornav_read_thread, NULL, read_lcm_vectornav, NULL);
        if(thread_rc) printf("[ERROR: RT Vectornav] Failed to create lcm read thread.\n");
        printf("[RT Vectornav] Done init for simulation.\n");
    }
    else
    {

        VnError error;
        VpeBasicControlRegister vpeReg;
        ImuFilteringConfigurationRegister filtReg;
        const char SENSOR_PORT[] = K_MINI_CHEETAH_VECTOR_NAV_SERIAL;
        const uint32_t SENSOR_BAUDRATE = 115200;
        char modelNumber[30];
        char strConversions[50];
        uint32_t newHz, oldHz;
        uint32_t hz_desired = 200;

        printf("[rt_vectornav] init_vectornav()\n");

        // initialize vectornav library
        VnSensor_initialize(&(vn.vs));

        // connect to sensor
        if ((error = VnSensor_connect(&(vn.vs), SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
        {
            printf("[rt_vectornav] VnSensor_connect failed.\n");
            processErrorReceived("Error connecting to sensor.", error);
            return;
        }

        // read the sensor's model number
        if ((error = VnSensor_readModelNumber(&(vn.vs), modelNumber, sizeof(modelNumber))) != E_NONE)
        {
            printf("[rt_vectornav] VnSensor_readModelNumber failed.\n");
            processErrorReceived("Error reading model number.", error);
            return;
        }
        printf("Model Number: %s\n", modelNumber);

        // switch the sensor to 1 kHz mode
        if ((error = VnSensor_readAsyncDataOutputFrequency(&(vn.vs), &oldHz)) != E_NONE)
        {
            printf("[rt_vectornav] VnSensor_readAsyncDataOutputFrequency failed.\n");
            processErrorReceived("Error reading async data output frequency.", error);
            return;
        }

        // non-zero frequency causes the IMU to output ascii packets at the set frequency, as well as binary
        if ((error = VnSensor_writeAsyncDataOutputFrequency(&(vn.vs), 0, true)) != E_NONE)
        {
            printf("[rt_vectornav] VnSensor_wrtieAsyncDataOutputFrequency failed.\n");
            processErrorReceived("Error writing async data output frequency.", error);
            return;
        }
        if ((error = VnSensor_readAsyncDataOutputFrequency(&(vn.vs), &newHz)) != E_NONE)
        {
            printf("[rt_vectornav] VnSensor_readAsyncDataOutputFrequency failed.\n");
            processErrorReceived("Error reading async data output frequency.", error);
            return;
        }
        printf("[rt_vectornav] Changed frequency from %d to %d Hz.\n",oldHz,newHz);

        // change to relative heading mode to avoid compass weirdness
        if ((error = VnSensor_readVpeBasicControl(&(vn.vs), &vpeReg)) != E_NONE)
        {
            printf("[rt_vectornav] VnSensor_ReadVpeBasicControl failed.\n");
            processErrorReceived("Error reading VPE basic control.", error);
            return;
        }
        strFromHeadingMode(strConversions, vpeReg.headingMode);
        printf("[rt_vectornav] Sensor was in mode: %s\n", strConversions);
        vpeReg.headingMode = VNHEADINGMODE_RELATIVE;
        if ((error = VnSensor_writeVpeBasicControl(&(vn.vs), vpeReg, true)) != E_NONE)
        {
            printf("[rt_vectornav] VnSensor_writeVpeBasicControl failed.\n");
            processErrorReceived("Error writing VPE basic control.", error);
            return;
        }
        if ((error = VnSensor_readVpeBasicControl(&(vn.vs), &vpeReg)) != E_NONE)
        {
            processErrorReceived("Error reading VPE basic control.", error);
            printf("[rt_vectornav] VnSensor_ReadVpeBasicControl failed.\n");
            return;
        }
        strFromHeadingMode(strConversions, vpeReg.headingMode);
        printf("[rt_vectornav] Sensor now id mode: %s\n", strConversions);


        if((error = VnSensor_readImuFilteringConfiguration(&(vn.vs), &filtReg)) != E_NONE)
        {
            printf("[rt_vectornav] VnSensor_readGyroCompensation failed.\n");
        }
        printf("[rt_vectornav] AccelWindow: %d\n", filtReg.accelWindowSize);
//        filtReg.accelWindowSize = 4;    // We're sampling at 200 hz, but the imu samples at 800 hz.
//        filtReg.accelFilterMode = 3;
//        if((error = VnSensor_writeImuFilteringConfiguration(&(vn.vs), filtReg, true)) != E_NONE)
//        {
//            printf("[rt_vectornav] VnSensor_writeGyroCompensation failed.\n");
//        }

        // setup binary output message type
        BinaryOutputRegister_initialize(
                    &(vn.bor),
                    ASYNCMODE_PORT2,
                    4,	//divisor:  output frequency = 800/divisor
                    COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
                    TIMEGROUP_NONE,
                    IMUGROUP_NONE,
                    GPSGROUP_NONE,
                    ATTITUDEGROUP_NONE,
                    INSGROUP_NONE);

        if ((error = VnSensor_writeBinaryOutput1(&(vn.vs), &(vn.bor), true)) != E_NONE)
        {
            printf("[rt_vectornav] VnSensor_writeBinaryOutput1 failed.\n");
            processErrorReceived("Error writing binary output 1.", error);
            return;
        }

        // setup handler
        VnSensor_registerAsyncPacketReceivedHandler(&(vn.vs),vectornav_handler,NULL);
        printf("[rt_vectornav] IMU is set up!\n");
    }
}

int got_first_vectornav_message = 0;
void vectornav_handler(void* userData, VnUartPacket *packet, size_t running_index)
{
    vec4f quat;
    vec3f omega;
    vec3f a;


    char strConversions[50];

    if(VnUartPacket_type(packet) != PACKETTYPE_BINARY)
    {
        printf("[vectornav_handler] got a packet that wasn't binary.\n");
        return;
    }

		if (!VnUartPacket_isCompatible(packet,
            COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
			GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE))
        {
            printf("[vectornav_handler] got a packet with the wrong type of data.\n");
            return;
        }

        quat  = VnUartPacket_extractVec4f(packet);
        omega = VnUartPacket_extractVec3f(packet);
        a     = VnUartPacket_extractVec3f(packet);

#ifdef PRINT_VECTORNAV_DEBUG
        str_vec4f(strConversions, quat);
        printf("[QUAT] %s\n",strConversions);
        
        str_vec3f(strConversions, omega);
        printf("[OMEGA] %s\n", strConversions);

        str_vec3f(strConversions, a);
        printf("[ACC] %s\n", strConversions);
#endif

        pthread_mutex_lock(&vectornav_data_mutex);
        for(int i = 0; i < 3; i++)
        {
            vectornav_data.acc[i] = a.c[i];
            vectornav_data.gyro[i] = omega.c[i];
            vectornav_data.quat[i] = quat.c[i];
        }
        vectornav_data.quat[3] = quat.c[3];
        pthread_mutex_unlock(&vectornav_data_mutex);

#ifdef LOG_VECTORNAV
        cheetahlcm_vectornav_data_t_publish(lcm_vectornav,
                                "SENSOR_vectornav_data",(cheetahlcm_vectornav_data_t*)&vectornav_data);
#endif
}

int processErrorReceived(char* errorMessage, VnError errorCode)
{
	char errorCodeStr[100];
	strFromVnError(errorCodeStr, errorCode);
	printf("%s\nERROR: %s\n", errorMessage, errorCodeStr);
	return -1;
}

cheetahlcm_vectornav_data_t get_vectornav()
{
    return vectornav_data;
}
