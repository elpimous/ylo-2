#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <map>

#include "myahrs_plus.hpp"
using namespace WithRobot;

static const int BAUDRATE = 115200;

static const char* DIVIDER = "1";  // 100 Hz


void handle_error(const char* error_msg)
{
    fprintf(stderr, "ERROR: %s\n", error_msg);
    exit(1);
}

void wait_for_user_input()
{
    printf("\npress enter key to quit.\n");
    char c = getchar();
    exit(1);
}

/******************************************************************************************************************************
 *
 *  EXAMPLE
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

    printf("%04d) sensor_id %d, Quaternion(xyzw)=%.4f,%.4f,%.4f,%.4f, Accel(xyz)=%.4f,%.4f,%.4f, \n\nGyro(xyz)=%.4f,%.4f,%.4f, Magnet(xyz)=%.2f,%.2f,%.2f\n",
            *counter,
            sensor_id,
            q.x, q.y, q.z, q.w,
            imu.ax, imu.ay, imu.az,
            imu.gx, imu.gy, imu.gz,
            imu.mx, imu.my, imu.mz);
}

void ex3_asynchronous_read_binary(const char* serial_device, int baudrate)
{
    printf("\n### %s() ###\n", __FUNCTION__);

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
        handle_error("start() returns false");
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

    while(sample_counter < 1000) {
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
 *
 *
 ******************************************************************************************************************************/

void usage(const char* prog)
{
    const char* dev_name_example = "/dev/ttyACM0 /dev/ttyACM1 ...";
    printf("\nusage : %s \"example_number\" \"serial port list\"  \n\t ex) $ %s 1 %s\n\n", prog, prog, dev_name_example);
}

void read_serial_devices_from_command_line(std::vector<std::string>& args, std::vector<std::string>& serial_device_list)
{
    for(size_t i=1; i<args.size(); i++) {
        serial_device_list.push_back(args[i]);
    }
}

int main(int argc, char* argv[]) {
    std::vector<std::string> args;
    for(int i=0; i<argc; i++) {
        args.push_back(std::string(argv[i]));
    }

    if(argc < 3) {
        printf("ERROR. need more arguments\n");
        usage(args[0].c_str());
        exit(1);
    }

    int example_id = atoi(args[1].c_str());

    std::vector<std::string> serial_device_list;
    for(size_t i=2; i<args.size(); i++) {
        serial_device_list.push_back(args[i]);
    }

    for(size_t i=0; i<serial_device_list.size(); i++) {
        printf(" - serial device(%d) : %s\n", i, serial_device_list[i].c_str());
    }

    switch(example_id) {
    case 3:
    	ex3_asynchronous_read_binary(serial_device_list[0].c_str(), BAUDRATE);
    	break;
    default:
    	handle_error("Invalid example id");
    	break;
    }

    wait_for_user_input();

    return 0;
}


