#pragma once
#include "v8stdint.h"

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#include <io.h>
#include <process.h>
#else
#include <pthread.h>
#include <assert.h>
#endif

#define UNUSED(x) (void)x

#if defined(__ANDROID__)
#define    pthread_cancel(x) 0
#endif

#define CLASS_THREAD(c , x ) Thread::ThreadCreateObjectFunctor<c, &c::x>(this)

class Thread
{
public:

	template <class CLASS, int (CLASS::*PROC)(void)> static Thread ThreadCreateObjectFunctor(CLASS * pthis){
		return createThread(createThreadAux<CLASS,PROC>, pthis);
	}

	template <class CLASS, int (CLASS::*PROC)(void) > static _size_t THREAD_PROC createThreadAux(void * param){
		return (static_cast<CLASS *>(param)->*PROC)();
	}

	static Thread createThread(thread_proc_t proc, void * param = NULL ){
		Thread thread_(proc, param);
#if defined(_WIN32)    
		thread_._handle = (_size_t)( _beginthreadex(NULL, 0, (unsigned int (__stdcall * )( void * ))proc, param, 0, NULL));
#else
		assert( sizeof(thread_._handle) >= sizeof(pthread_t));

		pthread_create((pthread_t *)&thread_._handle, NULL, (void * (*)(void *))proc, param);
#endif
		return thread_;
	}

public:
	explicit Thread(): _param(NULL),_func(NULL),_handle(0){}
	virtual ~Thread(){}
	_size_t getHandle(){ 
		return _handle;
	}
	int terminate(){
#if defined(_WIN32)   
		if (!this->_handle){ 
			return 0;
		}
		if (TerminateThread( reinterpret_cast<HANDLE>(this->_handle), -1)){
			CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
			this->_handle = NULL;
			return 0;
		}else{
			return -2;
		} 

#else
		if (!this->_handle) return 0;

		return pthread_cancel((pthread_t)this->_handle);
#endif
	}
	void *getParam() { 
		return _param;
	}
	int join(unsigned long timeout = -1){
		if (!this->_handle){ 
			return 0;
		}
#if defined(_WIN32) 
		switch ( WaitForSingleObject(reinterpret_cast<HANDLE>(this->_handle), timeout)){
		case WAIT_OBJECT_0:
			CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
			this->_handle = NULL;
			return 0;
		case WAIT_ABANDONED:
			return -2;
		case WAIT_TIMEOUT:
			return -1;
		}		
#else
		UNUSED(timeout);
		pthread_join((pthread_t)(this->_handle), NULL);
#endif
		return 0;
	}

	bool operator== ( const Thread & right) { 
		return this->_handle == right._handle; 
	}
protected:
	explicit Thread( thread_proc_t proc, void * param ):_param(param),_func(proc), _handle(0){}
	void * _param;
	thread_proc_t _func;
	_size_t _handle;
};

