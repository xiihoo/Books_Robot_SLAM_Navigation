#pragma once
#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#include <process.h>
#include <tlhelp32.h>
#include <sys/utime.h>
#include <io.h>
#include <direct.h>
#else
#include <assert.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#endif


class Locker
{
public:
	enum LOCK_STATUS {
		LOCK_OK = 0,
		LOCK_TIMEOUT = -1,
		LOCK_FAILED = -2
	};

	Locker(){
#ifdef _WIN32
		_lock = NULL;
#endif
		init();
	}

	~Locker(){
		release();
	}

	Locker::LOCK_STATUS lock(unsigned long timeout = 0xFFFFFFFF) {
#ifdef _WIN32
		switch (WaitForSingleObject(_lock, timeout==0xFFFFFFF?INFINITE:(DWORD)timeout)) {
		case WAIT_ABANDONED:
			return LOCK_FAILED;
		case WAIT_OBJECT_0:
			return LOCK_OK;
		case WAIT_TIMEOUT:
			return LOCK_TIMEOUT;
		}

#else
#ifdef _MACOS
		if (timeout !=0 ) {
			if (pthread_mutex_lock(&_lock) == 0) return LOCK_OK;
		}
#else
		if (timeout == 0xFFFFFFFF){
			if (pthread_mutex_lock(&_lock) == 0) return LOCK_OK;
		}
#endif
		else if (timeout == 0) {
			if (pthread_mutex_trylock(&_lock) == 0) return LOCK_OK;
		}
#ifndef _MACOS
		else{
			timespec wait_time;
			timeval now;
			gettimeofday(&now,NULL);

			wait_time.tv_sec = timeout/1000 + now.tv_sec;
			wait_time.tv_nsec = (timeout%1000)*1000000 + now.tv_usec*1000;

			if (wait_time.tv_nsec >= 1000000000){
				++wait_time.tv_sec;
				wait_time.tv_nsec -= 1000000000;
			}

#if !defined(__ANDROID__)

			switch (pthread_mutex_timedlock(&_lock,&wait_time)){
			case 0:
				return LOCK_OK;
			case ETIMEDOUT:
				return LOCK_TIMEOUT;
			}

#else
			struct timeval timenow;
			struct timespec sleepytime;
			/* This is just to avoid a completely busy wait */
			sleepytime.tv_sec = 0;
			sleepytime.tv_nsec = 10000000; /* 10ms */

			while (pthread_mutex_trylock (&_lock) == EBUSY) {
				gettimeofday (&timenow, NULL);

				if (timenow.tv_sec >= wait_time.tv_sec &&
					(timenow.tv_usec * 1000) >= wait_time.tv_nsec) {
						return LOCK_TIMEOUT;
				}

				nanosleep (&sleepytime, NULL);
			}

			return LOCK_OK;

#endif

		}
#endif
#endif

		return LOCK_FAILED;
	}


	void unlock(){
#ifdef _WIN32
		ReleaseMutex(_lock);
#else
		pthread_mutex_unlock(&_lock);
#endif
	}

#ifdef _WIN32
	HANDLE getLockHandle(){
		return _lock;
	}
#else
	pthread_mutex_t *getLockHandle(){
		return &_lock;
	}
#endif



protected:
	void init(){
#ifdef _WIN32
		_lock = CreateMutex(NULL,FALSE,NULL);
#else
		pthread_mutex_init(&_lock, NULL);
#endif
	}

	void release(){
		unlock();
#ifdef _WIN32

		if(_lock){ 
			CloseHandle(_lock);
		}
		_lock = NULL;
#else
		pthread_mutex_destroy(&_lock);
#endif
	}

#ifdef _WIN32
	HANDLE  _lock;
#else
	pthread_mutex_t _lock;
#endif
};


class Event
{
public:

	enum {
		EVENT_OK = 1,
		EVENT_TIMEOUT = 2,
		EVENT_FAILED = 0,
	};

    explicit Event(bool isAutoReset = true, bool isSignal = false)
#ifdef _WIN32
		: _event(NULL)
#else
		: _is_signalled(isSignal)
		, _isAutoReset(isAutoReset)
#endif
	{
#ifdef _WIN32
		_event = CreateEvent(NULL, isAutoReset?FALSE:TRUE, isSignal?TRUE:FALSE, NULL); 
#else
		pthread_mutex_init(&_cond_locker, NULL);
		pthread_cond_init(&_cond_var, NULL);
#endif
	}

	~ Event(){
		release();
	}

	void set( bool isSignal = true ){
		if (isSignal){
#ifdef _WIN32
			SetEvent(_event);
#else
			pthread_mutex_lock(&_cond_locker);

			if ( _is_signalled == false ){
				_is_signalled = true;
				pthread_cond_signal(&_cond_var);
			}
			pthread_mutex_unlock(&_cond_locker);
#endif
		}else{
#ifdef _WIN32
			ResetEvent(_event);
#else
			pthread_mutex_lock(&_cond_locker);
			_is_signalled = false;
			pthread_mutex_unlock(&_cond_locker);
#endif
		}
	}

	unsigned long wait( unsigned long timeout = 0xFFFFFFFF ){
#ifdef _WIN32
		switch (WaitForSingleObject(_event, timeout==0xFFFFFFF?INFINITE:(DWORD)timeout)){
		case WAIT_FAILED:
			return EVENT_FAILED;
		case WAIT_OBJECT_0:
			return EVENT_OK;
		case WAIT_TIMEOUT:
			return EVENT_TIMEOUT;
		}
		return EVENT_OK;
#else
		unsigned long ans = EVENT_OK;
		pthread_mutex_lock( &_cond_locker );

		if (!_is_signalled){
			if (timeout == 0xFFFFFFFF){
				pthread_cond_wait(&_cond_var,&_cond_locker);
			}else{
				timespec wait_time;
				timeval now;
				gettimeofday(&now,NULL);

				wait_time.tv_sec = timeout/1000 + now.tv_sec;
				wait_time.tv_nsec = (timeout%1000)*1000000ULL + now.tv_usec*1000;

				if (wait_time.tv_nsec >= 1000000000){
					++wait_time.tv_sec;
					wait_time.tv_nsec -= 1000000000;
				}
				switch (pthread_cond_timedwait(&_cond_var,&_cond_locker,&wait_time)){
				case 0:
					// signalled
					break;
				case ETIMEDOUT:
					// time up
					ans = EVENT_TIMEOUT;
					goto _final;
					break;
				default:
					ans = EVENT_FAILED;
					goto _final;
				}

			}
		}

		assert(_is_signalled);

		if (_isAutoReset){
			_is_signalled = false;
		}
_final:
		pthread_mutex_unlock( &_cond_locker );

		return ans;
#endif

	}
protected:

	void release() {
#ifdef _WIN32
		CloseHandle(_event);
#else
		pthread_mutex_destroy(&_cond_locker);
		pthread_cond_destroy(&_cond_var);
#endif
	}

#ifdef _WIN32
	HANDLE _event;
#else
	pthread_cond_t         _cond_var;
	pthread_mutex_t        _cond_locker;
	bool                   _is_signalled;
	bool                   _isAutoReset;
#endif
};

class ScopedLocker
{
public :
    explicit ScopedLocker(Locker &l): _binded(l) {
		_binded.lock();
	}

	void forceUnlock() {
		_binded.unlock();
	}
	~ScopedLocker() {
		_binded.unlock();
	}
	Locker & _binded;
};


