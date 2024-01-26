
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>


#define PropertyBuilderByName(type, name, access_permission)\
    access_permission:\
        type m_##name;\
    public:\
    inline void set##name(type v) {\
        m_##name = v;\
    }\
    inline type get##name() {\
        return m_##name;\
}\

#define DEG2RAD(x) ((x)*M_PI/180.)

class YDLIDAR_API CYdLidar
{
    PropertyBuilderByName(float,MaxRange,private)
    PropertyBuilderByName(float,MinRange,private)
    PropertyBuilderByName(float,MaxAngle,private)
    PropertyBuilderByName(float,MinAngle,private)
    PropertyBuilderByName(int,ScanFrequency,private)

    PropertyBuilderByName(bool,Intensities,private)
    PropertyBuilderByName(bool,FixedResolution,private)
    PropertyBuilderByName(bool,Exposure,private)
    PropertyBuilderByName(bool,HeartBeat,private)
    PropertyBuilderByName(bool,Reversion, private)

    PropertyBuilderByName(int,SerialBaudrate,private)
    PropertyBuilderByName(int,SampleRate,private)

    PropertyBuilderByName(std::string,SerialPort,private)
    PropertyBuilderByName(std::vector<float>,IgnoreArray,private)


public:
	CYdLidar(); //!< Constructor
	virtual ~CYdLidar();  //!< Destructor: turns the laser off.

    bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

    // Return true if laser data acquistion succeeds, If it's not
    bool doProcessSimple(LaserScan &outscan, bool &hardwareError);

    //Turn on the motor enable
	bool  turnOn();  //!< See base class docs
    //Turn off the motor enable and close the scan
	bool  turnOff(); //!< See base class docs

    /** Returns true if the device is in good health, If it's not*/
	bool getDeviceHealth() const;

    /** Returns true if the device information is correct, If it's not*/
    bool getDeviceInfo(int &type);

    /** Retruns true if the heartbeat function is set to heart is successful, If it's not*/
    bool checkHeartBeat() const;

    /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
    bool checkScanFrequency();

    //Turn off lidar connection
    void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

protected:
    /** Returns true if communication has been established with the device. If it's not,
      *  try to create a comms channel.
      * \return false on error.
      */
    bool  checkCOMMs();

    /** Returns true if health status and device information has been obtained with the device. If it's not,
      * \return false on error.
      */
    bool  checkStatus();

    /** Returns true if the normal scan runs with the device. If it's not,
      * \return false on error.
      */
    bool checkHardware();



private:
    bool isScanning;
    int node_counts ;
    double each_angle;
    int show_error;
};	// End of class

