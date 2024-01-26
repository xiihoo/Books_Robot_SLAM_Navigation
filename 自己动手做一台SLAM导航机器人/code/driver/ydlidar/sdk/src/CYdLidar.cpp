#include "CYdLidar.h"
#include "common.h"
#include <map>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar()
{
    m_SerialPort = "";
    m_SerialBaudrate = 115200;
    m_Intensities = false;
    m_FixedResolution = false;
    m_Exposure = false;
    m_HeartBeat = false;
    m_Reversion = false;
    m_MaxAngle = 180.f;
    m_MinAngle = -180.f;
    m_MaxRange = 16.0;
    m_MinRange = 0.08;
    m_SampleRate = 9;
    m_ScanFrequency = 7;
    isScanning = false;
    node_counts = 720;
    each_angle = 0.5;
    show_error = 0;
    m_IgnoreArray.clear();
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar()
{
    disconnecting();
}

void CYdLidar::disconnecting()
{
    if (YDlidarDriver::singleton()) {
        YDlidarDriver::singleton()->disconnect();
        YDlidarDriver::done();
    }
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError){
	hardwareError			= false;

	// Bound?
    if (!checkHardware())
	{
        hardwareError = true;
        return false;
	}

    node_info nodes[node_counts];
    size_t   count = _countof(nodes);

    size_t all_nodes_counts = node_counts;

    //  wait Scan data:
    uint64_t tim_scan_start = getTime();
	result_t op_result =  YDlidarDriver::singleton()->grabScanData(nodes, count);
    const uint64_t tim_scan_end = getTime();

	// Fill in scan data:
	if (op_result == RESULT_OK)
	{
		op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);
		//同步后的时间
		if(nodes[0].stamp > 0){
            tim_scan_start = nodes[0].stamp;
        }
		const double scan_time = tim_scan_end - tim_scan_start;
		if (op_result == RESULT_OK)
		{
            if(m_FixedResolution){
                all_nodes_counts = count;
            }
            each_angle = 360.0/all_nodes_counts;

            node_info angle_compensate_nodes[all_nodes_counts];
            memset(angle_compensate_nodes, 0, all_nodes_counts*sizeof(node_info));
            unsigned int i = 0;
            for( ; i < count; i++) {
                if (nodes[i].distance_q2 != 0) {
                    float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                    if(m_Reversion){
                       angle=angle+180;
                       if(angle>=360){ angle=angle-360;}
                        nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
                    }
                    int inter =(int)( angle / each_angle );
                    float angle_pre = angle - inter * each_angle;
                    float angle_next = (inter+1) * each_angle - angle;
                    if (angle_pre < angle_next) {
                        if(inter < all_nodes_counts)
                            angle_compensate_nodes[inter]=nodes[i];
                    } else {
                        if (inter < all_nodes_counts -1)
                            angle_compensate_nodes[inter+1]=nodes[i];
                    }
                }
             }

            LaserScan scan_msg;

            if (m_MaxAngle< m_MinAngle) {
                float temp = m_MinAngle;
                m_MinAngle = m_MaxAngle;
                m_MaxAngle = temp;
            }


            int counts = all_nodes_counts*((m_MaxAngle-m_MinAngle)/360.0f);
            int angle_start = 180+m_MinAngle;
            int node_start = all_nodes_counts*(angle_start/360.0f);

            scan_msg.ranges.resize(counts);
            scan_msg.intensities.resize(counts);
            float range = 0.0;
            float intensity = 0.0;
            int index = 0;


            for (size_t i = 0; i < all_nodes_counts; i++) {
                range = (float)angle_compensate_nodes[i].distance_q2/4.0f/1000;
                intensity = (float)(angle_compensate_nodes[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

                if (i<all_nodes_counts/2) {
                    index = all_nodes_counts/2-1-i;
                } else {
                    index =all_nodes_counts-1-(i-all_nodes_counts/2);
                }

                if (m_IgnoreArray.size() != 0) {
                    float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                    if (angle>180) {
                        angle=360-angle;
                    } else {
                        angle=-angle;
                    }

                    for (uint16_t j = 0; j < m_IgnoreArray.size();j = j+2) {
                        if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j+1])) {
                           range = 0.0;
                           break;
                        }
                    }
                }

                if (range > m_MaxRange|| range < m_MinRange) {
                    range = 0.0;
                }

                int pos = index - node_start ;
                if (0<= pos && pos < counts) {
                    scan_msg.ranges[pos] =  range;
                    scan_msg.intensities[pos] = intensity;
                }
            }

            scan_msg.system_time_stamp = tim_scan_start;
            scan_msg.self_time_stamp = tim_scan_start;
            scan_msg.config.min_angle = DEG2RAD(m_MinAngle);
            scan_msg.config.max_angle = DEG2RAD(m_MaxAngle);
            scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) / (double)counts;
            scan_msg.config.time_increment = scan_time / (double)counts;
            scan_msg.config.scan_time = scan_time;
            scan_msg.config.min_angle = m_MinRange;
            scan_msg.config.max_range = m_MaxRange;
            outscan = scan_msg;
            return true;


		}

    } else {
        if (op_result==RESULT_FAIL) {
			// Error? Retry connection
			//this->disconnect();
		}
	}

	return false;

}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn()
{
    bool ret = false;
    if (isScanning) {
		YDlidarDriver::singleton()->startMotor();
        ret = true;
	}

	return ret;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff()
{
	if (YDlidarDriver::singleton()) {
		YDlidarDriver::singleton()->stop();
		YDlidarDriver::singleton()->stopMotor();
        isScanning = false;
	}
	return true;
}

/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() const {
	if (!YDlidarDriver::singleton()) return false;

	result_t op_result;
    device_health healthinfo;

	op_result = YDlidarDriver::singleton()->getHealth(healthinfo);
    if (op_result == RESULT_OK) {
        printf("Yd Lidar running correctly ! The health status: %s\n", (int)healthinfo.status==0?"good":"bad");

        if (healthinfo.status == 2) {
            if (show_error == 3)
                fprintf(stderr, "Error, Yd Lidar internal error detected. Please reboot the device to retry.\n");
            return false;
        } else {
                return true;
        }

    } else {
        if (show_error == 3)
            fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
        return false;
    }

}

bool CYdLidar::getDeviceInfo(int &type) {

	if (!YDlidarDriver::singleton()) return false;

	device_info devinfo;
    if (YDlidarDriver::singleton()->getDeviceInfo(devinfo) != RESULT_OK ) {
        if (show_error == 3)
            fprintf(stderr, "get DeviceInfo Error\n" );
		return false;
	}	 
	std::string model;
    sampling_rate _rate;
    int _samp_rate=4;
    result_t ans;
    int bad = 0;

    type = devinfo.model;
    switch (devinfo.model) {
        case 1:
            model="F4";
            break;
        case 2:
            model="T1";
            break;
        case 3:
            model="F2";
            break;
        case 4:
            model="S4";
            break;
        case 5:
        {
            model="G4";
            ans = YDlidarDriver::singleton()->getSamplingRate(_rate);
            if (ans == RESULT_OK) {
                switch (m_SampleRate) {
                case 4:
                    _samp_rate=0;
                    break;
                case 8:
                    _samp_rate=1;
                    break;
                case 9:
                    _samp_rate=2;
                    break;
                default:
                    _samp_rate = _rate.rate;
                    break;
                }

                while (_samp_rate != _rate.rate) {
                    ans = YDlidarDriver::singleton()->setSamplingRate(_rate);
                    if (ans != RESULT_OK) {
                        bad++;
                        if(bad>5){
                            break;
                        }
                    }
                }

                switch (_rate.rate) {
                    case 0:
                        _samp_rate = 4;
                        break;
                    case 1:
                        node_counts = 1440;
                        each_angle = 0.25;
                        _samp_rate=8;
                        break;
                    case 2:
                        node_counts = 1440;
                        each_angle = 0.25;
                        _samp_rate=9;
                        break;
                }


            }

	    }
            break;
        case 6:
            model="X4";
        case 8:
        {
            model="F4Pro";
            ans = YDlidarDriver::singleton()->getSamplingRate(_rate);
            if (ans == RESULT_OK) {
                switch (m_SampleRate) {
                case 4:
                    _samp_rate=0;
                    break;
                case 6:
                    _samp_rate=1;
                    break;
                default:
                    _samp_rate = _rate.rate;
                    break;
                }
                while (_samp_rate != _rate.rate) {
                    ans = YDlidarDriver::singleton()->setSamplingRate(_rate);
                    if (ans != RESULT_OK) {
                        bad++;
                        if(bad>5){
                            break;
                        }
                    }
                }

                switch (_rate.rate) {
                    case 0:
                        _samp_rate = 4;
                        break;
                    case 1:
                        node_counts = 1440;
                        each_angle = 0.25;
                        _samp_rate=6;
                        break;
                }

            }

        }
            break;
        case 9:
            model = "G4C";
            break;
        default:
            model = "Unknown";
            break;
    }

    m_SampleRate = _samp_rate;



    unsigned int maxv = (unsigned int)(devinfo.firmware_version>>8);
    unsigned int midv = (unsigned int)(devinfo.firmware_version&0xff)/10;
    unsigned int minv = (unsigned int)(devinfo.firmware_version&0xff)%10;

	printf("[YDLIDAR] Connection established in [%s]:\n"
			   "Firmware version: %u.%u.%u\n"
			   "Hardware version: %u\n"
			   "Model: %s\n"
			   "Serial: ",
                m_SerialPort.c_str(),
			    maxv,
			    midv,
                minv,
			    (unsigned int)devinfo.hardware_version,
			    model.c_str());

		for (int i=0;i<16;i++)
			printf("%01X",devinfo.serialnum[i]&0xff);
		printf("\n");

        printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n" , _samp_rate);


        float freq = 7.0f;
        if (devinfo.model == 5 || devinfo.model ==8 || devinfo.model == 9) {
            checkScanFrequency();
            checkHeartBeat();
        } else {
            printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);
        }

		return true;
	

}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency()
{
    float freq = 7.0f;
    scan_frequency _scan_frequency;
    int hz = 0;
    if (5 <= m_ScanFrequency && m_ScanFrequency <= 12) {
        result_t ans = YDlidarDriver::singleton()->getScanFrequency(_scan_frequency) ;
        if (ans == RESULT_OK) {
            freq = _scan_frequency.frequency/100.f;
            hz = m_ScanFrequency - freq;
            if (hz>0) {
                while (hz != 0) {
                    YDlidarDriver::singleton()->setScanFrequencyAdd(_scan_frequency);
                    hz--;
                }
                freq = _scan_frequency.frequency/100.0f;
            } else {
                while (hz != 0) {
                    YDlidarDriver::singleton()->setScanFrequencyDis(_scan_frequency);
                    hz++;
                }
                freq = _scan_frequency.frequency/100.0f;
            }
        }
        if (m_ScanFrequency < 7 && m_SampleRate>6) {
            node_counts = 1600;

        } else if ( m_ScanFrequency < 6 && m_SampleRate == 9) {
            node_counts = 2000;

        } else if ( m_ScanFrequency < 6 && m_SampleRate == 4) {
            node_counts = 900;
        }
        each_angle = 360.0/node_counts;
    }

    printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);

    return true;

}

/*-------------------------------------------------------------
                        checkHeartBeat
-------------------------------------------------------------*/

bool CYdLidar::checkHeartBeat() const
{
    bool ret = false;
    scan_heart_beat beat;
    result_t ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
    if (m_HeartBeat) {
        if (beat.enable&& ans == RESULT_OK) {
            ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
        }
        if (!beat.enable&& ans == RESULT_OK ) {
            YDlidarDriver::singleton()->setHeartBeat(true);
            ret = true;
        }
    } else {
        if (!beat.enable&& ans == RESULT_OK) {
            ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
        }
        if (beat.enable && ans==RESULT_OK) {
            YDlidarDriver::singleton()->setHeartBeat(false);
            ret = true;
        }

    }

    return ret;

}
/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs()
{
    if (!YDlidarDriver::singleton()) {
        // create the driver instance
        YDlidarDriver::initDriver();
        if (!YDlidarDriver::singleton()) {
             fprintf(stderr, "Create Driver fail\n");
            return false;

        }

    }
    if (YDlidarDriver::singleton()->isconnected()) {
        return true;
    }

	// Is it COMX, X>4? ->  "\\.\COMX"
    if (m_SerialPort.size()>=3) {
        if ( tolower( m_SerialPort[0]) =='c' && tolower( m_SerialPort[1]) =='o' && tolower( m_SerialPort[2]) =='m' ) {
			// Need to add "\\.\"?
            if (m_SerialPort.size()>4 || m_SerialPort[3]>'4')
                m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
		}
	}

	// make connection...
    result_t op_result = YDlidarDriver::singleton()->connect(m_SerialPort.c_str(), m_SerialBaudrate);
    if (op_result != RESULT_OK) {
        fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified serial port %s\n",  m_SerialPort.c_str() );
		return false;
	}

	return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus()
{

    if (!YDlidarDriver::singleton())
        return false;
    if (YDlidarDriver::singleton()->isscanning())
        return true;

    std::map<int, bool> checkmodel;
    checkmodel.insert(std::map<int, bool>::value_type(115200, false));
    checkmodel.insert(std::map<int, bool>::value_type(128000, false));
    checkmodel.insert(std::map<int, bool>::value_type(153600, false));
    checkmodel.insert(std::map<int, bool>::value_type(230400, false));

    again:
    // check health:
    bool ret = getDeviceHealth();

    int m_type;
    if (!getDeviceInfo(m_type)&&!ret){
        checkmodel[m_SerialBaudrate] = true;
        map<int,bool>::iterator it;
        for (it=checkmodel.begin(); it!=checkmodel.end(); ++it) {
            if(it->second)
                continue;

            show_error++;
            YDlidarDriver::singleton()->disconnect();
            YDlidarDriver::done();
            YDlidarDriver::initDriver();
            if (!YDlidarDriver::singleton()) {
                printf("YDLIDAR Create Driver fail, exit\n");
                return false;
            }
            m_SerialBaudrate = it->first;

            bool ret = checkCOMMs();
            if (!ret) {
                return false;
            }
            goto again;
        }

        return false;
    }

    show_error = 0;
    m_Intensities = false;
    if (m_type == 4) {
        if (m_SerialBaudrate == 153600)
            m_Intensities = true;
        if (m_Intensities) {
            scan_exposure exposure;
            int cnt = 0;
            while ((YDlidarDriver::singleton()->setLowExposure(exposure) == RESULT_OK) && (cnt<3)) {
                if (exposure.exposure != m_Exposure) {
                        printf("set EXPOSURE MODEL SUCCESS!!!\n");
                        break;
                }
                cnt++;
            }
            if (cnt>=3) {

                fprintf(stderr,"set LOW EXPOSURE MODEL FALIED!!!\n");
            }
        }
    }

    YDlidarDriver::singleton()->setIntensities(m_Intensities);

     // start scan...
    result_t s_result= YDlidarDriver::singleton()->startScan();
    if (s_result != RESULT_OK) {
        fprintf(stderr, "[CYdLidar] Error starting scanning mode: %x\n", s_result);
        isScanning = false;
        return false;
    }
    printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
    fflush(stdout);
    fflush(stderr);
    isScanning = true;
    return true;

}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware()
{
    bool ret = true;
    if (!isScanning) {
        ret = false;
        if (checkCOMMs()) {
            if (checkStatus()) {
                if (turnOn()) {
                    ret = true;
                }
            }
        }
    }

    return ret;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize()
{
	bool ret = true;
    if (!checkCOMMs()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
        return false;
	}
    if (!checkStatus()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.because of failure in scan mode.\n");
    }
    if (!turnOn()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner. Because the motor falied to start.\n");
		
	}
    return ret;
	
}
