/*
 *  YDLIDAR SYSTEM
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include "laser_test.h"
using namespace serial;
using namespace ydlidar;
Lasertest::DEVICE_STATE Lasertest::device_state_ = CLOSED;
static int nodes_count = 720;
static float each_angle = 0.5;

Lasertest::Lasertest()
       :publish_freq_(40),
          scan_no_(0) {
    port_ = "/dev/ttyACM0";
    baudrate_ = 115200;
    angle_min_ = -180;
    angle_max_ = 180;
    intensities_ = false;
    inverted = false;
   
}

Lasertest::~Lasertest() {
}

void Lasertest::setPort(std::string port)
{
    port_ = port;
}

void Lasertest::setBaudrate(int baud)
{
    baudrate_ = baud;
}

void  Lasertest::setIntensities(bool intensities)
{
    intensities_ = intensities;
}


void Lasertest::run() {
    Open();
    Start();
}

void  Lasertest::closeApp(int signo){
    device_state_ = CLOSED;
    signal(signo, SIG_DFL);
}

/** Returns true if the device is connected & operative */
bool Lasertest::getDeviceInfo()
{
    if (!YDlidarDriver::singleton()){
        return false;
    }

    device_info devinfo;
    if (YDlidarDriver::singleton()->getDeviceInfo(devinfo) !=RESULT_OK){
        fprintf(stderr,"[YDLIDAR] get DeviceInfo Error\n" );
        return false;
    }
                
    sampling_rate _rate;   
    std::string model;
    switch(devinfo.model){
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
            model="G4";
                    YDlidarDriver::singleton()->getSamplingRate(_rate);
                    switch(_rate.rate){
                        case 0:
                            break;
                        case 1:
                            nodes_count = 1440;
                            each_angle = 0.25;
                            break;
                        case 2:
                            nodes_count = 1440;
                            each_angle = 0.25;
                            break;
                    }
            break;
        case 6:
            model="X4";
            break;
            case 8:
            model="F4Pro";
                    YDlidarDriver::singleton()->getSamplingRate(_rate);
                    switch(_rate.rate){
                        case 0:
                            break;
                        case 1:
                            nodes_count = 1440;
                            each_angle = 0.25;
                            break;
                    }
            break;
            case 9:
            model="G4C";
            break;
        default:
            model = "Unknown";
            break;
    }

    unsigned int maxv = (unsigned int)(devinfo.firmware_version>>8);
    unsigned int midv = (unsigned int)(devinfo.firmware_version&0xff)/10;
    unsigned int minv = (unsigned int)(devinfo.firmware_version&0xff)%10;

    printf("[YDLIDAR] Connection established in [%s]:\n"
			   "Firmware version: %u.%u.%u\n"
			   "Hardware version: %u\n"
			   "Model: %s\n"
			   "Serial: ",
			    port_.c_str(),
			    maxv,
			    midv,
                	    minv,
			    (unsigned int)devinfo.hardware_version,
			    model.c_str());

    for (int i=0;i<16;i++){
        printf("%01X",devinfo.serialnum[i]&0xff);
    }
    printf("\n");
    return true;

}

/** Returns true if the device is connected & operative */
bool Lasertest::getDeviceHealth()
{
    if (!YDlidarDriver::singleton()) return false;

    result_t op_result;
    device_health healthinfo;

    op_result = YDlidarDriver::singleton()->getHealth(healthinfo);
    if (op_result == RESULT_OK) { 
        fprintf(stdout,"[YDLIDAR] running correctly ! The health status:%s\n" ,healthinfo.status==0? "good":"bad");
        if (healthinfo.status == 2) {
            fprintf(stderr, "Error, [YDLIDAR] internal error detected. Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve YDLIDAR health code: %x\n", op_result);
        return false;
    }
}

void Lasertest::Open() {
    try {
	if(!YDlidarDriver::singleton()){
	    YDlidarDriver::initDriver();
        } 
	result_t op_result = YDlidarDriver::singleton()->connect(port_.c_str(), (uint32_t)baudrate_);
	if (op_result != RESULT_OK) {
	  fprintf(stdout,"open Lidar is failed! Exit!! ......\n");
	  return;
	}

	signal(SIGINT, closeApp); 
    	signal(SIGTERM, closeApp);

        bool ret = getDeviceHealth();
	if(!getDeviceInfo()&&!ret){
	    YDlidarDriver::singleton()->disconnect();
	    YDlidarDriver::done();
	     return;
	}
        
	result_t ans=YDlidarDriver::singleton()->startScan();
	if(ans != RESULT_OK){
	    fprintf(stdout,"start Lidar is failed! Exit!! ......\n");
	    YDlidarDriver::singleton()->disconnect();
	    YDlidarDriver::done();
	    return;	
	}

        YDlidarDriver::singleton()->setIntensities(intensities_);
        fprintf(stdout,"Device opened successfully.\n");
        device_state_ = OPENED;
    } catch (std::exception &e) {
        Close();
        fprintf(stdout,"can't open laser\n ");
    }
}

void Lasertest::Start() {    
    if(device_state_ !=OPENED|| !YDlidarDriver::singleton()){
        return;
    }
    node_info all_nodes[nodes_count];
    memset(all_nodes, 0, nodes_count*sizeof(node_info));
    fprintf(stdout,"Now YDLIDAR is scanning.\n");
    device_state_ = RUNNING;

    double scan_duration;
    result_t op_result;

    while (device_state_ == RUNNING) {
        try {
            node_info nodes[nodes_count];
            size_t   count = _countof(nodes);
            uint64_t start_scan_time = getms();
            op_result = YDlidarDriver::singleton()->grabScanData(nodes, count);
            uint64_t end_scan_time = getms();
            scan_duration = (end_scan_time - start_scan_time);

            if (op_result == RESULT_OK) {
                op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);
                if (op_result == RESULT_OK) {
                    memset(all_nodes, 0, nodes_count*sizeof(node_info));
                    for(size_t i =0 ; i < count; i++) {
                        if (nodes[i].distance_q2 != 0) {
                            float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                            int inter =(int)( angle / each_angle );
                            float angle_pre = angle - inter * each_angle;
                            float angle_next = (inter+1) * each_angle - angle;
                            if(angle_pre < angle_next){
                                if(inter < nodes_count)
                                        all_nodes[inter]=nodes[i];
                                }else{
                                    if(inter < nodes_count-1)
                                        all_nodes[inter+1]=nodes[i];

                                }
                       		}

                    }
		    publicScanData(all_nodes,start_scan_time,scan_duration,nodes_count,angle_min_,angle_max_,inverted);

                }

	    }else if(op_result == RESULT_FAIL){
	    }
		
        } catch (std::exception& e) {
            fprintf(stderr,"Exception thrown while starting YDLIDAR.\n ");
            break;
   	} catch (...) {
            fprintf(stderr,"Exception thrown while trying to get scan: \n");
	    break;
    	}
    }  

	Close(); 
}

void Lasertest::Stop() {
    device_state_ = OPENED;
}

void Lasertest::Close() {
    try {
        YDlidarDriver::singleton()->disconnect();
	YDlidarDriver::done();
        device_state_ = CLOSED;
	exit(0);
    } catch (std::exception &e) {
        return;
    }
}

std::vector<int> Lasertest::split(const std::string &s, char delim) {
    std::vector<int> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atoi(number.c_str()));
    }
    return elems;
}

void Lasertest::publicScanData(node_info *nodes, uint64_t start,double scan_time, size_t node_count, float angle_min,float angle_max, bool reverse_data) {
    fprintf(stdout,"publicScanData: %lud   ,  %i\n",start, (int)node_count);

    int counts = node_count*((angle_max-angle_min)/360.0f);
    int angle_start = 180+angle_min;
    int node_start = node_count*(angle_start/360.0f);

    LaserScan scan;
    scan.system_time_stamp = start;
    scan.self_time_stamp = start;

    float radian_min = DEG2RAD(angle_min);
    float radian_max = DEG2RAD(angle_max);

    
    scan.config.min_angle = radian_min;
    scan.config.max_angle = radian_max;
    scan.config.ang_increment = (radian_max - radian_min) / (double)counts;
    scan.config.time_increment = scan_time / (double)counts;
    scan.config.scan_time = scan_time;
    scan.config.min_angle = 0.1;
    scan.config.max_range = 15.;

    scan.ranges.resize(counts);
    scan.intensities.resize(counts);

    float range = 0.0;
    float intensity = 0.0;
    int index = 0;
    for (size_t i = 0; i < node_count; i++) {
	range = (float)nodes[i].distance_q2/4.0f/1000;
	intensity = (float)(nodes[i].sync_quality >> 2);

        if(i<node_count/2){
	    index = node_count/2-1-i;	    
        }else{
	    index =node_count-1-(i-node_count/2);
        }

	int pos = index - node_start ;
        if(0<= pos && pos < counts){
	    scan.ranges[pos] =  range;
	    scan.intensities[pos] = intensity;
	}

    }

    scan_no_++;

}
