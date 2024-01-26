#if !defined(_WIN32)

#ifndef SERIAL_IMPL_UNIX_H
#define SERIAL_IMPL_UNIX_H

#include <pthread.h>
#include <assert.h>
#include <termios.h>
#include "serial.h"

namespace serial {

	using std::size_t;
	using std::string;


	class MillisecondTimer {
	public:
        explicit MillisecondTimer(const uint32_t millis);
		int64_t remaining();

	private:
		static timespec timespec_now();
		timespec expiry;
	};

	class serial::Serial::SerialImpl {
	public:
        explicit SerialImpl (const string &port,
			unsigned long baudrate,
			bytesize_t bytesize,
			parity_t parity,
			stopbits_t stopbits,
			flowcontrol_t flowcontrol);

		virtual ~SerialImpl ();

		bool open ();

		void close ();

		bool isOpen () const;

		size_t available ();

		bool waitReadable (uint32_t timeout);

		void waitByteTimes (size_t count);

		int waitfordata(size_t data_count, uint32_t timeout, size_t * returned_size);

		size_t read (uint8_t *buf, size_t size = 1);

		size_t write (const uint8_t *data, size_t length);


		void flush ();

		void flushInput ();

		void flushOutput ();

		void sendBreak (int duration);

		bool setBreak (bool level);

		bool setRTS (bool level);

		bool setDTR (bool level);

		bool waitForChange ();

		bool getCTS ();

		bool getDSR ();

		bool getRI ();

		bool getCD ();

		uint32_t getByteTime();

		void setPort (const string &port);

		string getPort () const;

		void setTimeout (Timeout &timeout);

		Timeout getTimeout () const;

		bool setBaudrate (unsigned long baudrate);

		bool setStandardBaudRate(speed_t baudrate);

		bool setCustomBaudRate(unsigned long baudrate);

		unsigned long getBaudrate () const;

		bool setBytesize (bytesize_t bytesize);

		bytesize_t getBytesize () const;

		bool setParity (parity_t parity);

		parity_t getParity () const;

		bool setStopbits (stopbits_t stopbits);

		stopbits_t getStopbits () const;

		bool setFlowcontrol (flowcontrol_t flowcontrol);

		flowcontrol_t getFlowcontrol () const;

		bool setTermios(const termios *tio);

		bool getTermios(termios *tio);

		int readLock ();

		int readUnlock ();

		int writeLock ();

		int writeUnlock ();


	private:
		string port_;               // Path to the file descriptor
		int fd_;                    // The current file descriptor

		bool is_open_;
		bool xonxoff_;
		bool rtscts_;

		Timeout timeout_;           // Timeout for read operations
		unsigned long baudrate_;    // Baudrate
		uint32_t byte_time_ns_;     // Nanoseconds to transmit/receive a single byte

		parity_t parity_;           // Parity
		bytesize_t bytesize_;       // Size of the bytes
		stopbits_t stopbits_;       // Stop Bits
		flowcontrol_t flowcontrol_; // Flow Control

		// Mutex used to lock the read functions
		pthread_mutex_t read_mutex;
		// Mutex used to lock the write functions
		pthread_mutex_t write_mutex;
	};

}

#endif // SERIAL_IMPL_UNIX_H

#endif // !defined(_WIN32)
