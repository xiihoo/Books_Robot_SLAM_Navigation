#pragma once
#include "../sdk/include/ydlidar_driver.h"
#include "../sdk/src/common.h"
#include <exception>
#include <stdexcept>
#include <string>
#include <signal.h>

#define DEG2RAD(x) ((x)*M_PI/180.)

#define RAD2DEG(x) ((x)*180./M_PI)
#define NODE_COUNTS 720
#define EACH_ANGLE 0.5

const double PI = 3.1415926;

class Lasertest {
public:
    Lasertest();

    virtual ~Lasertest();

    void run();

    void setPort(std::string port);
    void setBaudrate(int baud);
    void setIntensities(bool intensities);
private:
    void Open();
    void Start();
    void Stop();
    void Close();
    static void closeApp(int signo);

    bool getDeviceHealth();
    bool getDeviceInfo();

    std::vector<int> split(const std::string &s, char delim);

    void publicScanData(node_info *nodes, uint64_t start,double scan_time, size_t node_count, float angle_min, float angle_max,bool reverse_data);


    enum DEVICE_STATE {
        OPENED,
        RUNNING,
        CLOSED,
    };

    static DEVICE_STATE device_state_;

    int scan_no_;

    std::string port_;
    int baudrate_;
    bool intensities_;
    int publish_freq_;
    double angle_min_, angle_max_;
    bool inverted;
};


