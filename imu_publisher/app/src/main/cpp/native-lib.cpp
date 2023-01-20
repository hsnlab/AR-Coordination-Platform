#include <jni.h>
#include <string>
#include <chrono>

//
// Created by nagyb on 2020. 05. 21..
//
#include <android/log.h>
#include "imu_manager.h"
#include <android/sensor.h>
#include <GLES2/gl2.h>
#include <dlfcn.h>
#include <dirent.h>
#include <thread>
#include <stdio.h>
#include <string.h>
#include <jni.h>
#include <map>
#include <unistd.h>

#define TAG "IMU-NATIVE"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)

struct AccelerometerData {
    GLfloat x;
    GLfloat y;
    GLfloat z;
};
struct GyroData {
    GLfloat rx;
    GLfloat ry;
    GLfloat rz;
};

const int LOOPER_ID_USER_A = 3; // accel
const int LOOPER_ID_USER_G = 4; // gyro
const int SENSOR_REFRESH_RATE_HZ = 450; // here you set the sampling rate of the IMU... for some reason you have to set something a bit below 500 Hz to get 500 Hz :)
constexpr int32_t SENSOR_REFRESH_PERIOD_US = int32_t(1000000 / SENSOR_REFRESH_RATE_HZ);

ASensorManager *sensorManager;
const ASensor *accelerometer;
ASensorEventQueue *accelerometerEventQueue;
const ASensor *gyro;
ASensorEventQueue *gyroEventQueue;
ALooper *looperA; // accel
ALooper *looperG; // gyro

AccelerometerData sensorDataA;
GyroData sensorDataG;

const long ncounter = 100;
long IMU0[ncounter];
float IMU1[ncounter];
float IMU2[ncounter];
float IMU3[ncounter];
float IMU4[ncounter];
float IMU5[ncounter];
float IMU6[ncounter];

long IMU0S = 0;
float IMU1S = 0.0;
float IMU2S = 0.0;
float IMU3S = 0.0;
float IMU4S = 0.0;
float IMU5S = 0.0;
float IMU6S = 0.0;

long counter = 0;

long MAXINTERVAL = 3000000;

int iter = 0;

int countERROR = 0;

long previous_timestampA = 0;
long previous_timestampG = 0;
long previous_timestampL = 0;

long time_count = 0;
long init_delay = 5000; // this value sets how long to wait before starting to record images
bool ready_to_start = false;
long timestamp_to_start = 0;
int flag_init_timestamp = 0;
struct timespec ts {
        0, 0
};
struct tm localTime;

int initialhour = 0;

int flag_init = 0;

static const char *kDirName = "/sdcard/X/";
static const char *kFileName = "capture";
FILE *file;

JNIEnv* jniEnv;
jobject my_jobject;
jclass clazz;
jmethodID sendMsg;

const char*  kPackageName = "com.android.accelerometergraph";
ASensorManager* AcquireASensorManagerInstance(void) {
    typedef ASensorManager *(*PF_GETINSTANCEFORPACKAGE)(const char *name);
    void* androidHandle = dlopen("libandroid.so", RTLD_NOW);
    PF_GETINSTANCEFORPACKAGE getInstanceForPackageFunc = (PF_GETINSTANCEFORPACKAGE)
            dlsym(androidHandle, "ASensorManager_getInstanceForPackage");
    if (getInstanceForPackageFunc) {
        return getInstanceForPackageFunc(kPackageName);
    }

    typedef ASensorManager *(*PF_GETINSTANCE)();
    PF_GETINSTANCE getInstanceFunc = (PF_GETINSTANCE)
            dlsym(androidHandle, "ASensorManager_getInstance");
    // by all means at this point, ASensorManager_getInstance should be available
    assert(getInstanceFunc);
    return getInstanceFunc();

//    ASensorManager* sensor_manager =
//            ASensorManager_getInstanceForPackage(kPackageName);
//    if (!sensor_manager) {
//        fprintf(stderr, "Failed to get a sensor manager\n");}
//
//    return sensor_manager;
}

void init_imu_file_creation(){

    DIR *dir = opendir(kDirName);
    if (dir) {
        closedir(dir);
    } else {
        std::string cmd = "mkdir -p ";
        cmd += kDirName;
        system(cmd.c_str());
    }


    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &localTime);

    std::string fileName = kDirName;
    std::string dash("-");
    fileName += kFileName + std::to_string(localTime.tm_mon) +
                std::to_string(localTime.tm_mday) + dash +
                std::to_string(localTime.tm_hour) +
                std::to_string(localTime.tm_min) +
                std::to_string(localTime.tm_sec) + ".csv";
    file = fopen(fileName.c_str(), "wb");

    initialhour = localTime.tm_hour;
}

void init_gyro(){
    LOGI("Initializing Gyro");

    sensorManager = AcquireASensorManagerInstance();
    assert(sensorManager != NULL);

    // Define the gyro
    gyro = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED); // make sure that you set uncalibrated
    assert(gyro != NULL);

    looperG = ALooper_forThread();
    if(looperG == NULL)
        looperG = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);

    gyroEventQueue = ASensorManager_createEventQueue(sensorManager, looperG,
                                                     LOOPER_ID_USER_G, NULL, NULL);
    assert(gyroEventQueue != NULL);

    auto statusG = ASensorEventQueue_enableSensor(gyroEventQueue,
                                                  gyro);
    assert(statusG >= 0);

    int sensor_delay = ASensor_getMinDelay(gyro);
    LOGI("Gyro sensor delay: %d", sensor_delay);


    statusG = ASensorEventQueue_setEventRate(gyroEventQueue,
                                             gyro,
                                             SENSOR_REFRESH_PERIOD_US);
    assert(statusG >= 0);
    (void)statusG;   //to silent unused compiler warning
}


/**
 * JA: Accel initialization
 */

static void init_accel()
{
    LOGI("Initializing Accelerometer");

    sensorManager = AcquireASensorManagerInstance();
    assert(sensorManager != NULL);

    // Define the accelerometer
    accelerometer = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED);
    assert(accelerometer != NULL);

    //looperA = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    //assert(looperA != NULL);

    looperA = ALooper_forThread();
    if(looperA == NULL)
        looperA = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);

    accelerometerEventQueue = ASensorManager_createEventQueue(sensorManager, looperA,
                                                              LOOPER_ID_USER_A, NULL, NULL);
    assert(accelerometerEventQueue != NULL);

    auto statusA = ASensorEventQueue_enableSensor(accelerometerEventQueue,
                                                  accelerometer);
    assert(statusA >= 0);

    int sensor_delay = ASensor_getMinDelay(accelerometer);
    LOGI("Accel sensor delay: %d", sensor_delay);

    statusA = ASensorEventQueue_setEventRate(accelerometerEventQueue,
                                             accelerometer,
                                             SENSOR_REFRESH_PERIOD_US);
    assert(statusA >= 0);
    (void)statusA;   //to silent unused compiler warning
}
ASensorEventQueue* queue;

int LOOPER_ID=1;

jfloat acc_x;
jfloat acc_y;
jfloat acc_z;
jlong timestamp;
jfloat gyro_x;
jfloat gyro_y;
jfloat gyro_z;
bool acc_ok=false;
bool gyro_ok=false;
int get_sensor_events(int fd, int events, void* data) {
    ASensorEvent event;

    while (ASensorEventQueue_getEvents(queue, &event, 1) > 0) {
        if (event.type == ASENSOR_TYPE_ACCELEROMETER) {
            LOGI("accl(x,y,z,t): %f %f %f",
                 event.acceleration.x, event.acceleration.y,
                 event.acceleration.z);

            acc_x=event.acceleration.x;
            acc_y= event.acceleration.y;
            acc_z= event.acceleration.z;
            timestamp= event.timestamp;
            acc_ok=true;
        } else if(event.type == ASENSOR_TYPE_GYROSCOPE){
            gyro_x=event.uncalibrated_gyro.x_uncalib;
            gyro_y=event.uncalibrated_gyro.y_uncalib;
            gyro_z=event.uncalibrated_gyro.z_uncalib;
            LOGI("gyro(x,x,y,y,z,z,t): %f %f %f %f %f %f",
                 event.uncalibrated_gyro.x_uncalib,gyro_x, event.uncalibrated_gyro.y_uncalib,gyro_y,
                 event.uncalibrated_gyro.z_uncalib,gyro_z);
            gyro_ok=true;
        }
        if(gyro_ok && acc_ok) {
            jniEnv->CallVoidMethod(my_jobject,sendMsg,timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z);
            gyro_ok= false;
            acc_ok=false;
        }
    }

    // Call the method on the object
   // jniEnv->CallVoidMethod(my_jobject,sendMsg,timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z);
    return 1;
}

void mainRun( ) {

    ASensorManager* sensorManager = AcquireASensorManagerInstance();
    ALooper* looper = ALooper_forThread();
    if(looper == NULL)
    looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);

    ASensorRef accelerometerSensor = ASensorManager_getDefaultSensor(sensorManager,ASENSOR_TYPE_ACCELEROMETER);
    ASensorRef gyroscopeSensor = ASensorManager_getDefaultSensor(sensorManager,ASENSOR_TYPE_GYROSCOPE);
    LOGI("accelerometerSensor: %s, vendor: %s", ASensor_getName(accelerometerSensor), ASensor_getVendor(accelerometerSensor));
    queue = ASensorManager_createEventQueue(sensorManager, looper, LOOPER_ID, get_sensor_events, NULL);

    ASensorEventQueue_enableSensor(queue, accelerometerSensor);
    ASensorEventQueue_setEventRate(queue, accelerometerSensor, (1000L/200)*1000);
    ASensorEventQueue_enableSensor(queue, gyroscopeSensor);
    ASensorEventQueue_setEventRate(queue, gyroscopeSensor, (1000L/200)*1000);
    int ident;//identifier

    int events;


}




void capture_all() {
    int ident;
    int events;

    int id= ALooper_pollOnce(10000,NULL,&events,NULL);
    ident=ALooper_pollAll(10000, NULL, &events, NULL);
    while(ident>= 0 ) {

        // Sample the gyro
        ASensorEvent eventG;
        while (ASensorEventQueue_getEvents(gyroEventQueue, &eventG, 1) > 0) {
            sensorDataG.rx = eventG.uncalibrated_gyro.x_uncalib;
            sensorDataG.ry = eventG.uncalibrated_gyro.y_uncalib;
            sensorDataG.rz = eventG.uncalibrated_gyro.z_uncalib;

            // check if we are not stuck in the same timestamp and recording several times here. Sometimes it happens for some reason..
            if (eventG.timestamp >= previous_timestampL) {

                long result = std::abs(eventG.timestamp - previous_timestampL);
                previous_timestampG = eventG.timestamp;
                previous_timestampL = eventG.timestamp;

                // check if for some reason we have missed a sample... never happens
                if (result > MAXINTERVAL) {
                    countERROR++;
                    LOGI("X G TS: %ld XXXX C %ld R %ld ERRORnum %d", (long) eventG.timestamp,
                         counter, result, countERROR);
                }

                if (time_count >= init_delay) {
                    // enter this function after init_delay since it takes some time to initialize the accelerometer for some reason..
                    // enter this condition the first time we record IMU data
                    // we record the timestamp from which we should be recording images

                    if (flag_init_timestamp == 0){ // logic to enter here once only
                        timestamp_to_start = eventG.timestamp;
                        ready_to_start = true;
                        flag_init_timestamp = 1; // never enter here again
                    }

                    //LOGI("X G TS: %ld XXXX %f", (long) eventG.timestamp, IMU4S);
                    if (result >= 1500000) { // this means it is a new measurement which means we save to file directly before logging the new measurement

                        if (flag_init == 1) {
                            fprintf(file, "%ld,%f,%f,%f,%f,%f,%f \n", IMU0S, IMU1S, IMU2S, // print to file
                                    IMU3S,
                                    IMU4S,
                                    IMU5S, IMU6S);
                        }
                        if (flag_init == 0) {
                            flag_init = 1;
                        }

                        // record the gyro values at this time step
                        IMU0S = eventG.timestamp;
                        IMU1S = sensorDataG.rx;
                        IMU2S = sensorDataG.ry;
                        IMU3S = sensorDataG.rz;

                    }
                    if (result < 1500000) { // this means it is the same measurement, which means we just log the value to be saved in the new iteration when both accel and gyro have been logged

                        // record the gyro values at this time step
                        IMU1S = sensorDataG.rx;
                        IMU2S = sensorDataG.ry;
                        IMU3S = sensorDataG.rz;
                    }
                }
                else{
                    time_count++;
                }
            }
        }

        // Sample the accelerometer
        ASensorEvent eventA;
        while (ASensorEventQueue_getEvents(accelerometerEventQueue, &eventA, 1) > 0) {
            sensorDataA.x = eventA.uncalibrated_gyro.x_uncalib;
            sensorDataA.y = eventA.uncalibrated_gyro.y_uncalib;
            sensorDataA.z = eventA.uncalibrated_gyro.z_uncalib;

            // check if we are not stuck in the same timestamp and recording several times here. Sometimes it happens for some reason..
            if (eventA.timestamp >= previous_timestampL) {

                long result = std::abs(eventA.timestamp - previous_timestampL);
                previous_timestampA = eventA.timestamp;
                previous_timestampL = eventA.timestamp;

                // check if for some reason we have missed a sample... never happens
                if (result > MAXINTERVAL) {
                    countERROR++;
                    LOGI("X A TS: %ld XXXX C %ld R %ld ERRORnum %d",
                         (long) eventA.timestamp,
                         counter, result, countERROR);
                }

                // enter this function after init_delay since it takes some time to initialize the accelerometer for some reason..
                // enter this condition the first time we record IMU data
                // we record the timestamp from which we should be recording images
                if (time_count >= init_delay) {

                    //LOGI("X A TS: %ld XXXX %f ", (long) eventG.timestamp, IMU1S);
                    if (result >= 1500000) { // this means it is a new measurement which means we save to file directly before logging the new measurement

                        if (flag_init == 1) {
                            fprintf(file, "%ld,%f,%f,%f,%f,%f,%f \n", IMU0S, IMU1S, IMU2S,
                                    IMU3S,
                                    IMU4S,
                                    IMU5S, IMU6S);
                        }
                        if (flag_init == 0) {
                            flag_init = 1;
                        }

                        IMU0S = eventA.timestamp;
                        IMU4S = sensorDataA.x;
                        IMU5S = sensorDataA.y;
                        IMU6S = sensorDataA.z;
                    }
                    if (result < 1500000) { // this means it is the same measurement, which means we just log the value to be saved in the new iteration when both accel and gyro have been logged
                        IMU4S = sensorDataA.x;
                        IMU5S = sensorDataA.y;
                        IMU6S = sensorDataA.z;
                    }
                    LOGI("X A TS: %ld",
                         (long) eventA.timestamp);

                }else{
                    time_count++;
                }
            }
        }

        // In case we want to stop recording after X seconds, minutes, hours. It is useful to record data over night.
        /*
         * clock_gettime(CLOCK_REALTIME, &ts);
        localtime_r(&ts.tv_sec, &localTime);

        //LOGI("initialhour %d current hours %d", initialhour, localTime.tm_hour);
        if ( localTime.tm_hour >= 9 && localTime.tm_min >= 30){
            //exit(EXIT_SUCCESS);
        }
         */

    }
}

extern "C" JNIEXPORT jstring JNICALL
Java_nbgy_bme_hsn_imu_1native_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

extern "C" JNIEXPORT jlong JNICALL
Java_nbgy_bme_hsn_imu_1native_MainActivity_setClass(
        JNIEnv* env,
        jobject /* this */) {

    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
const int kLooperId = 1;

extern "C" JNIEXPORT jlong JNICALL
Java_nbgy_bme_hsn_imu_1native_StreamerService_getTime(
        JNIEnv* env,
        jobject /* this */) {

    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void test( ) {
    ASensorManager* sensor_manager =
            ASensorManager_getInstanceForPackage(kPackageName);
    if (!sensor_manager) {
        LOGI("Failed to get a sensor manager");
        return;
    }
    ASensorList sensor_list;
    int sensor_count = ASensorManager_getSensorList(sensor_manager, &sensor_list);
    LOGI("Found %d sensors:", sensor_count);
    for (int i = 0; i < sensor_count; i++) {
        LOGI("Found %s", ASensor_getName(sensor_list[i]));
    }
    ASensorEventQueue* queue = ASensorManager_createEventQueue(
            sensor_manager,
            ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS),
            kLooperId, NULL /* no callback */, NULL /* no data */);
    if (!queue) {
        fprintf(stderr, "Failed to create a sensor event queue\n");
        return;
    }
    const int kTimeoutMicroSecs = 1000000;
    ASensorRef sensor = nullptr;
    bool sensor_found = false;
    for (int i = 0; i < sensor_count; i++) {
        sensor = sensor_list[i];
        int t= ASensorEventQueue_enableSensor(queue, sensor);
        if (t < 0)
            continue;
        if (ASensorEventQueue_setEventRate(queue, sensor, kTimeoutMicroSecs) < 0) {
            fprintf(stderr, "Failed to set the %s sample rate\n",
                    ASensor_getName(sensor));
            return;
        }
    }

    const std::map<int, std::string> kSensorSamples = {
            /*
             * Accelerometer:
             *   Reporting mode: continuous. Events are generated continuously.
             */
            { ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED, "accelerometer" },
            /*
             * Proximity sensor:
             *   Reporting mode: on-change. Events are generated only when proximity
             *     value has changed.
             */
            { ASENSOR_TYPE_PROXIMITY, "proximity sensor" },
            /*
             * Significant motion sensor:
             *   Reporting mode: one-shot. An event is generated when significant
             *     motion is detected. After that, the sensor will be disabled by
             *     itself.
             */
            { ASENSOR_TYPE_SIGNIFICANT_MOTION, "significant motion sensor" },
    };

    const int kNumSamples = 10;
    const int kNumEvents = 1;
    const int kTimeoutMilliSecs = 1000;
    const int kWaitTimeSecs = 1;
    for (auto& sensor_type : kSensorSamples) {
        const ASensor* sensor = ASensorManager_getDefaultSensor(sensor_manager,
                                                                sensor_type.first);
        if (sensor && !ASensorEventQueue_enableSensor(queue, sensor)) {
            for (int i = 0; i < kNumSamples; i++) {
                int ident = ALooper_pollAll(kTimeoutMilliSecs,
                                            NULL /* no output file descriptor */,
                                            NULL /* no output event */,
                                            NULL /* no output data */);
                if (ident == kLooperId) {
                    ASensorEvent data;
                    if (ASensorEventQueue_getEvents(queue, &data, kNumEvents)) {
                        if (sensor_type.first == ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
                            LOGI("Acceleration: x = %f, y = %f, z = %f\n",
                                   data.acceleration.x, data.acceleration.y,
                                   data.acceleration.z);
                        } else if (sensor_type.first == ASENSOR_TYPE_PROXIMITY) {
                            LOGI("Proximity distance: %f\n", data.distance);
                        } else if (sensor_type.first == ASENSOR_TYPE_SIGNIFICANT_MOTION) {
                            if (data.data[0] == 1) {
                                LOGI("Significant motion detected\n");
                                break;
                            }
                        }
                    }
                }
                sleep(kWaitTimeSecs);
            }
            int ret = ASensorEventQueue_disableSensor(queue, sensor);
            if (ret) {
                fprintf(stderr, "Failed to disable %s: %s\n",
                        sensor_type.second.c_str(), strerror(ret));
            }
        } else {
            fprintf(stderr, "No %s found or failed to enable it\n",
                    sensor_type.second.c_str());
        }
    }
    int ret = ASensorManager_destroyEventQueue(sensor_manager, queue);
    if (ret) {
        LOGI( "Failed to destroy event queue: %s\n", strerror(ret));
        return;
    }
    return;
}


void imu_thread() {
    LOGI("IMU thread!");
    init_imu_file_creation();
    init_accel();
    init_gyro();
    capture_all();
}

extern "C" JNIEXPORT void JNICALL
Java_nbgy_bme_hsn_imu_1native_StreamerService_startImu(
        JNIEnv* env, jobject thiz) {
//    std::thread thread (imu_thread);
    //imu_thread();
    jniEnv = env;
    my_jobject= env->NewGlobalRef(thiz);
    clazz = jniEnv->FindClass("nbgy/bme/hsn/imu_native/StreamerService");
    // Get the method that you want to call
    sendMsg = jniEnv->GetMethodID(clazz, "sendImuData", "(JFFFFFF)V");

    mainRun();



    return;
}


extern "C" JNIEXPORT void JNICALL
Java_nbgy_bme_hsn_imu_1native_StreamerService_stopImu(JNIEnv* env, jobject thiz){
    env->DeleteGlobalRef(my_jobject);
}
