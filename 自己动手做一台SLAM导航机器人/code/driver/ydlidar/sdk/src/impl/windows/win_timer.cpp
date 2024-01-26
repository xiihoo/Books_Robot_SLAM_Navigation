#if defined(_WIN32)
#include "timer.h"
#include <mmsystem.h>
#pragma comment(lib, "Winmm.lib")

namespace impl{

	static LARGE_INTEGER _current_freq;

	void HPtimer_reset() {
		BOOL ans=QueryPerformanceFrequency(&_current_freq);
       		_current_freq.QuadPart/=1000;
	}

    uint32_t getHDTimer() {
		LARGE_INTEGER current;
		QueryPerformanceCounter(&current);

        return (uint32_t)(current.QuadPart/(_current_freq.QuadPart));
	}

    TTimeStamp getCurrentTime(){
        FILETIME		t;
        GetSystemTimeAsFileTime(&t);
        return ((((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime))*100;
    }
	

	BEGIN_STATIC_CODE(timer_cailb) {
		HPtimer_reset();
	}END_STATIC_CODE(timer_cailb)

}
#endif
