#include "laser_test.h"
#include <iostream>
#include <string>
#include <signal.h>
using namespace std;

int main(int argc, char * argv[])
{
    bool showHelp  = argc>1 && !strcmp(argv[1],"--help");
    printf(" YDLIDAR C++ TEST\n");
	// Process arguments:
	if (argc<4 || showHelp ){
        printf("Usage: %s <serial_port> <baudrate> <intensities>\n\n",argv[0]);
        printf("Example:%s /dev/ttyUSB0 115200 0\n\n",argv[0]);
		if (!showHelp){				
            return -1;
        }else{
            return 0;
        }
	}

    const std::string port = string(argv[1]);
    const int baud =  atoi(argv[2]);
    const int intensities =  atoi(argv[3]);

    Lasertest laser;
    laser.setPort(port);
    laser.setBaudrate(baud);
    laser.setIntensities(intensities);
    laser.run();

    return 0;
}
