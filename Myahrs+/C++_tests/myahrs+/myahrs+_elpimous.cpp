#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <map>

#include "myahrs_plus.hpp"
using namespace WithRobot;

static const char* SERIAL_DEVICE = "/dev/ttyACM0";

static const int BAUDRATE = 115200;

static const char* DIVIDER = "1";  // 100 Hz

void handle_error(const char* error_msg)
{
    fprintf(stderr, "ERROR: %s\n", error_msg);
    exit(1);
}


/******************************************************************************************************************************
 *
 *  EXEMPLE
 *
 ******************************************************************************************************************************/
void ex3_callback_attribute(void* context, int sensor_id, const char* attribute_name, const char* value)
{
    printf(" ## sensor_id %d, Attribute has been changed(%s, %s)\n", sensor_id, attribute_name, value);
}

void ex3_callback_data(void* context, int sensor_id, SensorData* sensor_data)
{
    int* counter = (int*)context;
    (*counter)++;

    Quaternion& q = sensor_data->quaternion;
    ImuData<float>& imu = sensor_data->imu;

/*
    printf("%04d) sensor_id %d, Quaternion(xyzw)=%.4f,%.4f,%.4f,%.4f, Accel(xyz)=%.4f,%.4f,%.4f, \n\nGyro(xyz)=%.4f,%.4f,%.4f, Magnet(xyz)=%.2f,%.2f,%.2f\n",
            *counter,
            sensor_id,
            q.x, q.y, q.z, q.w,
            imu.ax, imu.ay, imu.az,
            imu.gx, imu.gy, imu.gz,
            imu.mx, imu.my, imu.mz);
            */
    printf("Gyro(xyz)=\nx: %.4f,\ny: %.4f,\nz: %.4f\n",
            imu.ax, imu.ay, imu.az);
}

void my_exemple(const char* serial_device, int baudrate)
{
    printf("\nSTARTING IMU FEEDBACK...\n");

    MyAhrsPlus sensor;

    int sample_counter = 0;

    /*
     * 	register a callback function to attribute changed event.
     */
    sensor.register_attribute_callback(ex3_callback_attribute, 0);

    /*
     * 	register a callback function to new data arrived event.
     */
    sensor.register_data_callback(ex3_callback_data, &sample_counter);

    /*
     * 	start communication with the myAHRS+.
     */
    if(sensor.start(serial_device, baudrate) == false) {
        handle_error("start() returns false, check ADRESS !!!");
    }

    /*
     *  set binary output format
     *   - select Quaternion and IMU data
     */
    if(sensor.cmd_binary_data_format("QUATERNION, IMU") == false) {
        handle_error("cmd_binary_data_format() returns false");
    }

    /*
     *  set divider
     *   - output rate(Hz) = max_rate/divider
     */
    if(sensor.cmd_divider(DIVIDER) ==false) {
        handle_error("cmd_divider() returns false");
    }

    /*
     *  set transfer mode
     *   - BC : BINARY Message & Continuous mode
     */
    if(sensor.cmd_mode("BC") ==false) {
        handle_error("cmd_mode() returns false");
    }

    while(sample_counter < 1000) { // 10s
        Platform::msleep(100);
    }

    /*
     * 	stop communication
     */
    sensor.stop();

    printf("END OF TEST(%s)\n\n", __FUNCTION__);
}


/******************************************************************************************************************************
 *
 *  RUN
 *
 ******************************************************************************************************************************/

int main(int argc, char* argv[]) {

    my_exemple(SERIAL_DEVICE, BAUDRATE);

    return 0;
}


