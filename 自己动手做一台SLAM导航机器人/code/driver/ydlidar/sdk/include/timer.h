#pragma once
#include "v8stdint.h"
#include <assert.h>
#include <time.h>
#include <inttypes.h>



#define BEGIN_STATIC_CODE( _blockname_ ) \
	static class _static_code_##_blockname_ {   \
	public:     \
	_static_code_##_blockname_ ()


#define END_STATIC_CODE( _blockname_ ) \
	}   _instance_##_blockname_;


#if defined(_WIN32)  
#include <windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <sys/time.h>
#include <unistd.h>

static inline void delay(uint32_t ms){
	while (ms>=1000){
		usleep(1000*1000);
		ms-=1000;
	};
	if (ms!=0){
		usleep(ms*1000);
	}
}
#endif





static inline TTimeStamp  time_tToTimestamp(const time_t &t )
{
    return uint64_t(t) * UINT64_C(10000000) +  UINT64_C(116444736) * UINT64_C(1000000000);
}

static inline TTimeStamp  time_tToTimestamp(const double t )
{
    return uint64_t(t*10000000.0)+ UINT64_C(116444736)*UINT64_C(1000000000);
}

static inline double timestampTotime_t( const TTimeStamp t )
{
    return double(t - UINT64_C(116444736)*UINT64_C(1000000000)) / 10000000.0;
}

static inline TTimeStamp timestampAdd( const TTimeStamp tim, const double num_seconds)
{
    return static_cast<TTimeStamp>(tim + static_cast<int64_t>(num_seconds*10000000.0));
}

/*---------------------------------------------------------------
                    timeDifference
  ---------------------------------------------------------------*/
static inline double timeDifference( const TTimeStamp t1, const TTimeStamp t2 )
{
    assert(t1!=INVALID_TIMESTAMP);
    assert(t2!=INVALID_TIMESTAMP);

    return (int64_t(t2)- int64_t(t1))/10000000.0;
}

/*---------------------------------------------------------------
                    secondsToTimestamp
  ---------------------------------------------------------------*/
static inline TTimeStamp secondsToTimestamp( const double nSeconds )
{
    return (TTimeStamp)(nSeconds*10000000.0);
}

namespace impl{

#if defined(_WIN32)  
	void HPtimer_reset();
#endif
    uint32_t getHDTimer();
    TTimeStamp getCurrentTime();
}


#define getms() impl::getHDTimer()
#define getTime() impl::getCurrentTime()
