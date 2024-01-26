#if !defined(_WIN32)

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#if !defined(__ANDROID__)
#include <sys/signal.h>
#include <sysexits.h>

#endif

#include <errno.h>
#include <paths.h>
#include <sys/param.h>
#include <pthread.h>
#include <poll.h>
#include <sys/utsname.h>

#include <asm/ioctls.h>

#if defined(__linux__) &&!defined(__ANDROID__)
# include <linux/serial.h>
#endif

#include <sys/select.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <time.h>

#ifdef __MACH__
#include <AvailabilityMacros.h>
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#include "unix_serial.h"

#ifndef TIOCINQ
#ifdef FIONREAD
#define TIOCINQ FIONREAD
#else
#define TIOCINQ 0x541B
#endif
#endif

#if defined(MAC_OS_X_VERSION_10_3) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_3)
#include <IOKit/serial/ioss.h>
#endif

/** 
* setup_port - Configure the port, eg. baud rate, data bits,etc.
* 
* @param fd        : The serial port
* @param speed     : The baud rate
* @param data_bits : The data bits
* @param parity    : The parity bits
* @param stop_bits : The stop bits
* 
* @return Return 0 if everything is OK, otherwise -1 with some error msg.
* @note
Here are termios structure members:
\verbatim
Member      Description 
c_cflag     Control options 
c_lflag     Line options 
c_iflag     Input options 
c_oflag     Output options 
c_cc        Control characters 
c_ispeed    Input baud (new interface) 
c_ospeed    Output baud (new interface) 
\endverbatim
The c_cflag member controls the baud rate, number of data bits, parity, 
stop bits, and hardware flow control. There are constants for all of the 
supported configurations.
Constant Description
\verbatim
CBAUD	Bit mask for baud rate 
B0	0 baud (drop DTR) 
B50	50 baud 
B75	75 baud 
B110	110 baud 
B134	134.5 baud 
B150	150 baud 
B200	200 baud 
B300	300 baud 
B600	600 baud 
B1200	1200 baud 
B1800	1800 baud 
B2400	2400 baud 
B4800	4800 baud 
B9600	9600 baud 
B19200	19200 baud 
B38400	38400 baud 
B57600	57,600 baud 
B76800	76,800 baud 
B115200	115,200 baud 
EXTA	External rate clock 
EXTB	External rate clock 
CSIZE	Bit mask for data bits 
CS5 5	data bits 
CS6 6	data bits 
CS7 7	data bits 
CS8 8	data bits 
CSTOPB	2 stop bits (1 otherwise) 
CREAD	Enable receiver 
PARENB	Enable parity bit 
PARODD	Use odd parity instead of even 
HUPCL	Hangup (drop DTR) on last close 
CLOCAL	Local line - do not change "owner" of port 
LOBLK	Block job control output 
CNEW_RTSCTS CRTSCTS	Enable hardware flow control (not supported on all 
platforms) 
\endverbatim
The input modes member c_iflag controls any input processing that is done to 
characters received on the port. Like the c_cflag field, the final value 
stored in c_iflag is the bitwise OR of the desired options.
\verbatim
Constant	Description 
INPCK	Enable parity check 
IGNPAR	Ignore parity errors 
PARMRK	Mark parity errors 
ISTRIP	Strip parity bits 
IXON	Enable software flow control (outgoing) 
IXOFF	Enable software flow control (incoming) 
IXANY	Allow any character to start flow again 
IGNBRK	Ignore break condition 
BRKINT	Send a SIGINT when a break condition is detected 
INLCR	Map NL to CR 
IGNCR	Ignore CR 
ICRNL	Map CR to NL 
IUCLC	Map uppercase to lowercase 
IMAXBEL	Echo BEL on input line too long 
\endverbatim
Here are some examples of setting parity checking: @n
No parity (8N1): 
\verbatim
options.c_cflag &= ~PARENB
options.c_cflag &= ~CSTOPB
options.c_cflag &= ~CSIZE;
options.c_cflag |= CS8;
\endverbatim
Even parity (7E1): 
\verbatim
options.c_cflag |= PARENB
options.c_cflag &= ~PARODD
options.c_cflag &= ~CSTOPB
options.c_cflag &= ~CSIZE;
options.c_cflag |= CS7;
\endverbatim
Odd parity (7O1): 
\verbatim
options.c_cflag |= PARENB
options.c_cflag |= PARODD
options.c_cflag &= ~CSTOPB
options.c_cflag &= ~CSIZE;
options.c_cflag |= CS7;
\endverbatim
*/


namespace serial{

	using std::string;
	using serial::MillisecondTimer;
	using serial::Serial;
	using namespace serial;

#define SNCCS 19


	struct termios2 {
		tcflag_t c_iflag;       /* input mode flags */
		tcflag_t c_oflag;       /* output mode flags */
		tcflag_t c_cflag;       /* control mode flags */
		tcflag_t c_lflag;       /* local mode flags */
		cc_t c_line;            /* line discipline */
		cc_t c_cc[SNCCS];          /* control characters */
		speed_t c_ispeed;       /* input speed */
		speed_t c_ospeed;       /* output speed */
	};

#ifndef TCGETS2
#define TCGETS2     _IOR('T', 0x2A, struct termios2)
#endif

#ifndef TCSETS2
#define TCSETS2     _IOW('T', 0x2B, struct termios2)
#endif

#ifndef BOTHER
#  define BOTHER      0010000
#endif


#if defined(__ANDROID__)
struct serial_struct {
    int     type;
    int     line;
    unsigned int    port;
    int     irq;
    int     flags;
    int     xmit_fifo_size;
    int     custom_divisor;
    int     baud_base;
    unsigned short  close_delay;
    char    io_type;
    char    reserved_char[1];
    int     hub6;
    unsigned short  closing_wait;
    unsigned short  closing_wait2;
    unsigned char   *iomem_base;
    unsigned short  iomem_reg_shift;
    unsigned int    port_high;
    unsigned long   iomap_base;
};
#    define ASYNC_SPD_CUST  0x0030
#    define ASYNC_SPD_MASK  0x1030
#    define PORT_UNKNOWN    0
#    define FNDELAY         0x800
#endif


	MillisecondTimer::MillisecondTimer (const uint32_t millis) : expiry(timespec_now()){
		int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);
		if (tv_nsec >= 1e9) {
			int64_t sec_diff = tv_nsec / static_cast<int> (1e9);
			expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
			expiry.tv_sec += sec_diff;
		} else {
			expiry.tv_nsec = tv_nsec;
		}
	}

	int64_t MillisecondTimer::remaining () {
		timespec now(timespec_now());
		int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;
		millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;
		return millis;
	}

	timespec MillisecondTimer::timespec_now () {
		timespec time;
# ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
		clock_serv_t cclock;
		mach_timespec_t mts;
		host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
		clock_get_time(cclock, &mts);
		mach_port_deallocate(mach_task_self(), cclock);
		time.tv_sec = mts.tv_sec;
		time.tv_nsec = mts.tv_nsec;
# else
		clock_gettime(CLOCK_MONOTONIC, &time);
# endif
		return time;
	}

	timespec timespec_from_ms (const uint32_t millis) {
		timespec time;
		time.tv_sec = millis / 1e3;
		time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
		return time;
	}


	static inline void set_common_props(termios *tio) {
#ifdef OS_SOLARIS
		tio->c_iflag &= ~(IMAXBEL|IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
		tio->c_oflag &= ~OPOST;
		tio->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
		tio->c_cflag &= ~(CSIZE|PARENB);
		tio->c_cflag |= CS8;
#else
		::cfmakeraw(tio);
#endif
		tio->c_cflag |= CLOCAL|CREAD;
		tio->c_cc[VTIME] = 0;
		tio->c_cc[VMIN] = 0;
	}

	
	static inline void set_databits(termios *tio, serial::bytesize_t databits) {
		tio->c_cflag &= ~CSIZE;
		switch (databits) {
		case serial::fivebits:
			tio->c_cflag |= CS5;
			break;
		case serial::sixbits:
			tio->c_cflag |= CS6;
			break;
		case serial::sevenbits:
			tio->c_cflag |= CS7;
			break;
		case serial::eightbits:
			tio->c_cflag |= CS8;
			break;
		default:
			tio->c_cflag |= CS8;
			break;
		}
	}


	static inline void set_parity(termios *tio, serial::parity_t parity) {
		tio->c_iflag &= ~(PARMRK | INPCK);
		tio->c_iflag |= IGNPAR;

		switch (parity) {

#ifdef CMSPAR
			// Here Installation parity only for GNU/Linux where the macro CMSPAR.
		case serial::parity_space:
			tio->c_cflag &= ~PARODD;
			tio->c_cflag |= PARENB | CMSPAR;
			break;
		case serial::parity_mark:
			tio->c_cflag |= PARENB | CMSPAR | PARODD;
			break;
#endif
		case serial::parity_none:
			tio->c_cflag &= ~PARENB;
			break;
		case serial::parity_even:
			tio->c_cflag &= ~PARODD;
			tio->c_cflag |= PARENB;
			break;
		case serial::parity_odd:
			tio->c_cflag |= PARENB | PARODD;
			break;
		default:
			tio->c_cflag |= PARENB;
			tio->c_iflag |= PARMRK | INPCK;
			tio->c_iflag &= ~IGNPAR;
			break;
		}
	}


	static inline void set_stopbits(termios *tio, serial::stopbits_t stopbits) {
		switch (stopbits) {
		case serial::stopbits_one:
			tio->c_cflag &= ~CSTOPB;
			break;
		case serial::stopbits_two:
			tio->c_cflag |= CSTOPB;
			break;
		default:
			tio->c_cflag &= ~CSTOPB;
			break;
		}
	}

	static inline void set_flowcontrol(termios *tio, serial::flowcontrol_t flowcontrol) {
		switch (flowcontrol) {
		case serial::flowcontrol_none:
			tio->c_cflag &= ~CRTSCTS;
			tio->c_iflag &= ~(IXON | IXOFF | IXANY);
			break;
		case serial::flowcontrol_hardware:
			tio->c_cflag |= CRTSCTS;
			tio->c_iflag &= ~(IXON | IXOFF | IXANY);
			break;
		case serial::flowcontrol_software:
			tio->c_cflag &= ~CRTSCTS;
			tio->c_iflag |= IXON | IXOFF | IXANY;
			break;
		default:
			tio->c_cflag &= ~CRTSCTS;
			tio->c_iflag &= ~(IXON | IXOFF | IXANY);
			break;
		}
	}


	static inline bool is_standardbaudrate(unsigned long baudrate, speed_t &baud){
		// setup baud rate
		bool custom_baud = false;
		switch (baudrate) {
#ifdef B0
		case 0: baud = B0; break;
#endif
#ifdef B50
		case 50: baud = B50; break;
#endif
#ifdef B75
		case 75: baud = B75; break;
#endif
#ifdef B110
		case 110: baud = B110; break;
#endif
#ifdef B134
		case 134: baud = B134; break;
#endif
#ifdef B150
		case 150: baud = B150; break;
#endif
#ifdef B200
		case 200: baud = B200; break;
#endif
#ifdef B300
		case 300: baud = B300; break;
#endif
#ifdef B600
		case 600: baud = B600; break;
#endif
#ifdef B1200
		case 1200: baud = B1200; break;
#endif
#ifdef B1800
		case 1800: baud = B1800; break;
#endif
#ifdef B2400
		case 2400: baud = B2400; break;
#endif
#ifdef B4800
		case 4800: baud = B4800; break;
#endif
#ifdef B7200
		case 7200: baud = B7200; break;
#endif
#ifdef B9600
		case 9600: baud = B9600; break;
#endif
#ifdef B14400
		case 14400: baud = B14400; break;
#endif
#ifdef B19200
		case 19200: baud = B19200; break;
#endif
#ifdef B28800
		case 28800: baud = B28800; break;
#endif
#ifdef B57600
		case 57600: baud = B57600; break;
#endif
#ifdef B76800
		case 76800: baud = B76800; break;
#endif
#ifdef B38400
		case 38400: baud = B38400; break;
#endif
#ifdef B115200
		case 115200: baud = B115200; break;
#endif
#ifdef B128000
		case 128000: baud = B128000; break;
#endif
#ifdef B153600
		case 153600: baud = B153600; break;
#endif
#ifdef B230400
		case 230400: baud = B230400; break;
#endif
#ifdef B256000
		case 256000: baud = B256000; break;
#endif
#ifdef B460800
		case 460800: baud = B460800; break;
#endif
#ifdef B576000
		case 576000: baud = B576000; break;
#endif
#ifdef B921600
		case 921600: baud = B921600; break;
#endif
#ifdef B1000000
		case 1000000: baud = B1000000; break;
#endif
#ifdef B1152000
		case 1152000: baud = B1152000; break;
#endif
#ifdef B1500000
		case 1500000: baud = B1500000; break;
#endif
#ifdef B2000000
		case 2000000: baud = B2000000; break;
#endif
#ifdef B2500000
		case 2500000: baud = B2500000; break;
#endif
#ifdef B3000000
		case 3000000: baud = B3000000; break;
#endif
#ifdef B3500000
		case 3500000: baud = B3500000; break;
#endif
#ifdef B4000000
		case 4000000: baud = B4000000; break;
#endif
		default:
			custom_baud = true;
		}

		return !custom_baud;

	}



	Serial::SerialImpl::SerialImpl (const string &port, unsigned long baudrate,
		bytesize_t bytesize,
		parity_t parity, stopbits_t stopbits,
		flowcontrol_t flowcontrol)
		: port_ (port), fd_ (-1), is_open_ (false), xonxoff_ (false), rtscts_ (false),
		baudrate_ (baudrate), parity_ (parity),
		bytesize_ (bytesize), stopbits_ (stopbits), flowcontrol_ (flowcontrol)
	{
		pthread_mutex_init(&this->read_mutex, NULL);
		pthread_mutex_init(&this->write_mutex, NULL);

	}

	Serial::SerialImpl::~SerialImpl () {
		close();
		pthread_mutex_destroy(&this->read_mutex);
		pthread_mutex_destroy(&this->write_mutex);
	}

	bool Serial::SerialImpl::open () {
		if (port_.empty ()) {
			return false;
		}
		if (is_open_ == true) {
			return true;
		}

		fd_ = ::open (port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_APPEND | O_NDELAY);

		if (fd_ == -1) {
			switch (errno) {
			case EINTR:
				// Recurse because this is a recoverable error.
				return open ();
			case ENFILE:
			case EMFILE:
			default:
				return false;
			}
		}

		termios tio;
		if (!getTermios(&tio)){
			return false;
		}
		set_common_props(&tio);
		set_databits(&tio, bytesize_);
		set_parity(&tio, parity_);
		set_stopbits(&tio, stopbits_);
		set_flowcontrol(&tio, flowcontrol_);

		if (!setTermios(&tio)){
			return false;
		}

		if (!setBaudrate(baudrate_)){
			return false;
		}

		// Update byte_time_ based on the new settings.
		uint32_t bit_time_ns = 1e9 / baudrate_;
		byte_time_ns_ = bit_time_ns * (1 + bytesize_ + parity_ + stopbits_);

		// Compensate for the stopbits_one_point_five enum being equal to int 3,
		// and not 1.5.
		if (stopbits_ == stopbits_one_point_five) {
			byte_time_ns_ += ((1.5 - stopbits_one_point_five) * bit_time_ns);
		}

		is_open_ = true;
		return true;
	}
	

	void Serial::SerialImpl::close (){
		if (is_open_ == true) {
			if (fd_ != -1) {
                ::close (fd_);
			}
			fd_ = -1;
			is_open_ = false;
		}
	}

	bool Serial::SerialImpl::isOpen () const {
		return is_open_;
	}

	size_t Serial::SerialImpl::available () {
		if (!is_open_) {
			return 0;
		}
		int count = 0;
		if (-1 == ioctl (fd_, TIOCINQ, &count)) {
			return 0;
		} else {
			return static_cast<size_t> (count);
		}
	}

	bool Serial::SerialImpl::waitReadable (uint32_t timeout) {
		// Setup a select call to block for serial data or a timeout
		fd_set readfds;
		FD_ZERO (&readfds);
		FD_SET (fd_, &readfds);
		timespec timeout_ts (timespec_from_ms (timeout));
		int r = pselect (fd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

		if (r < 0) {
			// Select was interrupted
			if (errno == EINTR) {
				return false;
			}
			// Otherwise there was some error
			return false;
		}
		// Timeout occurred
		if (r == 0) {
			return false;
		}
		// This shouldn't happen, if r > 0 our fd has to be in the list!
		if (!FD_ISSET (fd_, &readfds)) {
			return false;
		}
		// Data available to read.
		return true;
	}


	int Serial::SerialImpl::waitfordata(size_t data_count, uint32_t timeout, size_t * returned_size) {
		size_t length = 0;
		if (returned_size==NULL){
			returned_size=(size_t *)&length;
		}
		*returned_size = 0;

		int max_fd;
		fd_set input_set;
		struct timeval timeout_val;

		/* Initialize the input set */
		FD_ZERO(&input_set);
		FD_SET(fd_, &input_set);
		max_fd = fd_ + 1;

		/* Initialize the timeout structure */
		timeout_val.tv_sec = timeout / 1000;
		timeout_val.tv_usec = (timeout % 1000) * 1000;

		if(is_open_){
			if ( ioctl(fd_, FIONREAD, returned_size) == -1){
				return -2;     
			}
			if (*returned_size >= data_count) {
				return 0;
			}
		}

		MillisecondTimer total_timeout(timeout);

		while(is_open_){
			int64_t timeout_remaining_ms = total_timeout.remaining();
			if ((timeout_remaining_ms <= 0)) {
				// Timed out
				return -1;
			}
			/* Do the select */
			int n = ::select(max_fd, &input_set, NULL, NULL, &timeout_val);

			if(n < 0){
				if (errno == EINTR){
					return -1;
				}
				// Otherwise there was some error
				return -2;
			}else if(n == 0){
				// time out
				return -1;
			}else{
				// data avaliable
				assert (FD_ISSET(fd_, &input_set));

				if ( ioctl(fd_, FIONREAD, returned_size) == -1){
					return -2;
				}
				if (*returned_size >= data_count) {
					return 0;
				}else{
					int remain_timeout = timeout_val.tv_sec*1000000 + timeout_val.tv_usec;
					int expect_remain_time = (data_count - *returned_size)*1000000*8/baudrate_;
					if (remain_timeout > expect_remain_time){
						usleep(expect_remain_time);
					}
				}
			}
		}
		return -2;		
	}


	void Serial::SerialImpl::waitByteTimes (size_t count){
		timespec wait_time = { 0, static_cast<long>(byte_time_ns_ * count)};
		pselect (0, NULL, NULL, NULL, &wait_time, NULL);
	}

	size_t Serial::SerialImpl::read (uint8_t *buf, size_t size){
		// If the port is not open, throw
		if (!is_open_) {
			return 0;
		}
		size_t bytes_read = 0;

		// Calculate total timeout in milliseconds t_c + (t_m * N)
		long total_timeout_ms = timeout_.read_timeout_constant;
		total_timeout_ms += timeout_.read_timeout_multiplier * static_cast<long> (size);
		MillisecondTimer total_timeout(total_timeout_ms);

		// Pre-fill buffer with available bytes
		{
			ssize_t bytes_read_now = ::read (fd_, buf, size);
			if (bytes_read_now > 0) {
				bytes_read = bytes_read_now;
			}
		}

		while (bytes_read < size) {
			int64_t timeout_remaining_ms = total_timeout.remaining();
			if (timeout_remaining_ms <= 0) {
				// Timed out
				break;
			}
			// Timeout for the next select is whichever is less of the remaining
			// total read timeout and the inter-byte timeout.
			uint32_t timeout = std::min(static_cast<uint32_t> (timeout_remaining_ms), timeout_.inter_byte_timeout);
			// Wait for the device to be readable, and then attempt to read.
			if (waitReadable(timeout)) {
				// If it's a fixed-length multi-byte read, insert a wait here so that
				// we can attempt to grab the whole thing in a single IO call. Skip
				// this wait if a non-max inter_byte_timeout is specified.
				if (size > 1 && timeout_.inter_byte_timeout == Timeout::max()) {
					size_t bytes_available = available();
					if (bytes_available + bytes_read < size) {
						waitByteTimes(size - (bytes_available + bytes_read));
					}
				}
				// This should be non-blocking returning only what is available now
				//  Then returning so that select can block again.
				ssize_t bytes_read_now = ::read (fd_, buf + bytes_read, size - bytes_read);
				// read should always return some data as select reported it was
				// ready to read when we get to this point.
				if (bytes_read_now < 1) {
					// Disconnected devices, at least on Linux, show the
					// behavior that they are always ready to read immediately
					// but reading returns nothing.
					continue;
				}
				// Update bytes_read
				bytes_read += static_cast<size_t> (bytes_read_now);
				// If bytes_read == size then we have read everything we need
				if (bytes_read == size) {
					break;
				}
				// If bytes_read < size then we have more to read
				if (bytes_read < size) {
					continue;
				}
				// If bytes_read > size then we have over read, which shouldn't happen
				if (bytes_read > size) {
					break;
				}
			}
		}
		return bytes_read;
	}

	size_t Serial::SerialImpl::write (const uint8_t *data, size_t length) {
		if (is_open_ == false) {
			return 0;
		}
		fd_set writefds;
		size_t bytes_written = 0;

		// Calculate total timeout in milliseconds t_c + (t_m * N)
		long total_timeout_ms = timeout_.write_timeout_constant;
		total_timeout_ms += timeout_.write_timeout_multiplier * static_cast<long> (length);
		MillisecondTimer total_timeout(total_timeout_ms);

		bool first_iteration = true;
		while (bytes_written < length) {
			int64_t timeout_remaining_ms = total_timeout.remaining();
			// Only consider the timeout if it's not the first iteration of the loop
			// otherwise a timeout of 0 won't be allowed through
			if (!first_iteration && (timeout_remaining_ms <= 0)) {
				// Timed out
				break;
			}
			first_iteration = false;

			timespec timeout(timespec_from_ms(timeout_remaining_ms));

			FD_ZERO (&writefds);
			FD_SET (fd_, &writefds);

			// Do the select
			int r = pselect (fd_ + 1, NULL, &writefds, NULL, &timeout, NULL);

			// Figure out what happened by looking at select's response 'r'
			/** Error **/
			if (r < 0) {
				// Select was interrupted, try again
				if (errno == EINTR) {
					continue;
				}
				// Otherwise there was some error
				continue;
			}
			/** Timeout **/
			if (r == 0) {
				break;
			}
			/** Port ready to write **/
			if (r > 0) {
				// Make sure our file descriptor is in the ready to write list
				if (FD_ISSET (fd_, &writefds)) {
					// This will write some
					ssize_t bytes_written_now = ::write (fd_, data + bytes_written, length - bytes_written);
					// write should always return some data as select reported it was
					// ready to write when we get to this point.
					if (bytes_written_now < 1) {
						// Disconnected devices, at least on Linux, show the
						// behavior that they are always ready to write immediately
						// but writing returns nothing.
						continue;
					}
					// Update bytes_written
					bytes_written += static_cast<size_t> (bytes_written_now);
					// If bytes_written == size then we have written everything we need to
					if (bytes_written == length) {
						break;
					}
					// If bytes_written < size then we have more to write
					if (bytes_written < length) {
						continue;
					}
					// If bytes_written > size then we have over written, which shouldn't happen
					if (bytes_written > length) {
						break;
					}
				}
				// This shouldn't happen, if r > 0 our fd has to be in the list!
				break;
				//THROW (IOException, "select reports ready to write, but our fd isn't in the list, this shouldn't happen!");
			}
		}
		return bytes_written;
	}

	void Serial::SerialImpl::setPort (const string &port) {
		port_ = port;
	}

	string Serial::SerialImpl::getPort () const {
		return port_;
	}

	void Serial::SerialImpl::setTimeout (serial::Timeout &timeout) {
		timeout_ = timeout;
	}

	serial::Timeout Serial::SerialImpl::getTimeout () const {
		return timeout_;
	}

	bool Serial::SerialImpl::setBaudrate (unsigned long baudrate) {

		if(fd_==-1){
			return false;
		}
		baudrate_ = baudrate;
		// OS X support
#if defined(MAC_OS_X_VERSION_10_4) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_4)
		// Starting with Tiger, the IOSSIOSPEED ioctl can be used to set arbitrary baud rates
		// other than those specified by POSIX. The driver for the underlying serial hardware
		// ultimately determines which baud rates can be used. This ioctl sets both the input
		// and output speed.
		speed_t new_baud = static_cast<speed_t> (baudrate);
		if (-1 == ioctl (fd_, IOSSIOSPEED, &new_baud, 1)) {
			return false;
		}
		// Linux Support
#elif defined(__linux__) && defined (TIOCSSERIAL)
		speed_t baud;
		bool standard_baud = is_standardbaudrate(baudrate, baud);
		if(!standard_baud){
			return setCustomBaudRate(baudrate);
		}else{
			return setStandardBaudRate(baud);
		}
#else
		return false;
#endif

	}

	unsigned long Serial::SerialImpl::getBaudrate () const {
		return baudrate_;
	}


	bool Serial::SerialImpl::setStandardBaudRate(speed_t baudrate){
#ifdef __linux__
		// try to clear custom baud rate, using termios v2
		struct termios2 tio2;
		if (::ioctl(fd_, TCGETS2, &tio2) != -1) {
			if (tio2.c_cflag & BOTHER) {
				tio2.c_cflag &= ~BOTHER;
				tio2.c_cflag |= CBAUD;
				::ioctl(fd_, TCSETS2, &tio2);
			}
		}

		// try to clear custom baud rate, using serial_struct (old way)
		struct serial_struct serial;
		::memset(&serial, 0, sizeof(serial));
		if (::ioctl(fd_, TIOCGSERIAL, &serial) != -1) {
			if (serial.flags & ASYNC_SPD_CUST) {
				serial.flags &= ~ASYNC_SPD_CUST;
				serial.custom_divisor = 0;
				// we don't check on errors because a driver can has not this feature
				::ioctl(fd_, TIOCSSERIAL, &serial);
			}
		}
#endif

		termios tio;
		if (!getTermios(&tio)){
			return false;
		}
#ifdef _BSD_SOURCE
		if(::cfsetspeed(&tio, baudrate) <0){
			return false;
		}
#else
		if (::cfsetispeed(&tio, baudrate) < 0) {
			return false;
		}

		if (::cfsetospeed(&tio, baudrate) < 0) {
			return false;
		}
#endif
		return setTermios(&tio);
	}


	bool Serial::SerialImpl::setCustomBaudRate(unsigned long baudrate) {  
		struct termios2 tio2;

		if (::ioctl(fd_, TCGETS2, &tio2) != -1) {
			tio2.c_cflag &= ~CBAUD;
			tio2.c_cflag |= BOTHER;

			tio2.c_ispeed = baudrate;
			tio2.c_ospeed = baudrate;

			tcflush(fd_,TCIFLUSH); 

			if (fcntl(fd_, F_SETFL, FNDELAY)) {
				return false;
			}

			if (::ioctl(fd_, TCSETS2, &tio2) != -1 && ::ioctl(fd_, TCGETS2, &tio2) != -1) {
				return true;
			}
		}

		struct serial_struct serial;

		if (::ioctl(fd_, TIOCGSERIAL, &serial) == -1) {
			return false;
		}

		serial.flags &= ~ASYNC_SPD_MASK;
		serial.flags |= (ASYNC_SPD_CUST /* | ASYNC_LOW_LATENCY*/);
		serial.custom_divisor = serial.baud_base / baudrate;

		if (serial.custom_divisor == 0) {
			return false;
		}

		if (serial.custom_divisor * baudrate != serial.baud_base) {
		}

		if (::ioctl(fd_, TIOCSSERIAL, &serial) == -1) {
			return false;
		}

		return setStandardBaudRate(B38400);
	}
	

	bool Serial::SerialImpl::setBytesize (serial::bytesize_t bytesize) {
		termios tio;
		if (!getTermios(&tio)){
			return false;
		}
		bytesize_ = bytesize;
		set_databits(&tio, bytesize);

		return setTermios(&tio);
	}

	serial::bytesize_t Serial::SerialImpl::getBytesize () const {
		return bytesize_;
	}

	bool Serial::SerialImpl::setParity (serial::parity_t parity){
		termios tio;
		if (!getTermios(&tio)){
			return false;
		}
		parity_ = parity;
		set_parity(&tio, parity);

		return setTermios(&tio);
	}

	serial::parity_t Serial::SerialImpl::getParity () const{
		return parity_;
	}

	bool Serial::SerialImpl::setStopbits (serial::stopbits_t stopbits) {
		termios tio;
		if (!getTermios(&tio)){
			return false;
		}
		stopbits_ = stopbits;
		set_stopbits(&tio, stopbits);

		return setTermios(&tio);
	}

	serial::stopbits_t Serial::SerialImpl::getStopbits () const{
		return stopbits_;
	}

	bool Serial::SerialImpl::setFlowcontrol (serial::flowcontrol_t flowcontrol){
		termios tio;
		if (!getTermios(&tio)){
			return false;
		}
		flowcontrol_ = flowcontrol;
		set_flowcontrol(&tio, flowcontrol);

		return setTermios(&tio);
	}

	serial::flowcontrol_t Serial::SerialImpl::getFlowcontrol () const{
		return flowcontrol_;
	}


	bool Serial::SerialImpl::setTermios(const termios *tio){

		tcflush(fd_,TCIFLUSH); 

		if (fcntl(fd_, F_SETFL, FNDELAY)) {
			return false;
		}

		if (::tcsetattr(fd_, TCSANOW, tio) == -1) {
			return false;
		}
		return true;
	}

	bool Serial::SerialImpl::getTermios(termios *tio){
		::memset(tio, 0, sizeof(termios));
		if (::tcgetattr(fd_, tio) == -1) {
			return false;
		}
		return true;
	}

	void Serial::SerialImpl::flush (){
		if (is_open_ == false) {
			return;
		}
		#if !defined(__ANDROID__)
  		tcdrain (fd_);
		#endif
	}

	void Serial::SerialImpl::flushInput (){
		if (is_open_ == false) {
			return;
		}
		tcflush (fd_, TCIFLUSH);
	}

	void Serial::SerialImpl::flushOutput (){
		if (is_open_ == false) {
			return;
		}
		tcflush (fd_, TCOFLUSH);
	}

	void Serial::SerialImpl::sendBreak (int duration) {
		if (is_open_ == false) {
			return;
		}
		tcsendbreak (fd_, static_cast<int> (duration / 4));
	}

	bool Serial::SerialImpl::setBreak (bool level) {
		if (is_open_ == false) {
			return false;
		}

		if (level) {
			if (-1 == ioctl (fd_, TIOCSBRK)) {
				return false;
			}
		} else {
			if (-1 == ioctl (fd_, TIOCCBRK)) {
				return false;
			}
		}
		return true;
	}

	bool Serial::SerialImpl::setRTS (bool level){
		if (is_open_ == false) {
			return false;
		}

		int command = TIOCM_RTS;

		if (level) {
			if (-1 == ioctl (fd_, TIOCMBIS, &command)) {
				return false;
			}
		} else {
			if (-1 == ioctl (fd_, TIOCMBIC, &command)) {
				return false;
			}
		}
		return true;
	}

	bool Serial::SerialImpl::setDTR (bool level) {
		if (is_open_ == false) {
			return false;
		}

		int command = TIOCM_DTR;

		if (level) {
			if (-1 == ioctl (fd_, TIOCMBIS, &command)) {
				return false;
			}
		} else {
			if (-1 == ioctl (fd_, TIOCMBIC, &command)) {
				return false;
			}
		}
		return true;
	}

	bool Serial::SerialImpl::waitForChange () {
#ifndef TIOCMIWAIT

		while (is_open_ == true) {

			int status;

			if (-1 == ioctl (fd_, TIOCMGET, &status)) {
				return false;
			}else{
				if (0 != (status & TIOCM_CTS)
					|| 0 != (status & TIOCM_DSR)
					|| 0 != (status & TIOCM_RI)
					|| 0 != (status & TIOCM_CD))
				{
					return true;
				}
			}

			usleep(1000);
		}

		return false;
#else
		int command = (TIOCM_CD|TIOCM_DSR|TIOCM_RI|TIOCM_CTS);

		if (-1 == ioctl (fd_, TIOCMIWAIT, &command)) {
			return false;
		}
		return true;
#endif
	}

	bool Serial::SerialImpl::getCTS (){
		if (is_open_ == false) {
			return false;
		}

		int status;

		if (-1 == ioctl (fd_, TIOCMGET, &status)){
			return false;
		}else{
			return 0 != (status & TIOCM_CTS);
		}
	}

	bool Serial::SerialImpl::getDSR (){
		if (is_open_ == false) {
			return false;
		}

		int status;

		if (-1 == ioctl (fd_, TIOCMGET, &status)){
			return false;
		}else{
			return 0 != (status & TIOCM_DSR);
		}
	}

	bool Serial::SerialImpl::getRI (){
		if (is_open_ == false) {
			return false;
		}

		int status;

		if (-1 == ioctl (fd_, TIOCMGET, &status)){
			return false;
		}else{
			return 0 != (status & TIOCM_RI);
		}
	}

	bool Serial::SerialImpl::getCD (){
		if (is_open_ == false) {
			return false;
		}

		int status;

		if (-1 == ioctl (fd_, TIOCMGET, &status)){
			return false;
		}else{
			return 0 != (status & TIOCM_CD);
		}
	}

	uint32_t Serial::SerialImpl::getByteTime(){
		return byte_time_ns_;
	}

	int Serial::SerialImpl::readLock (){
		int result = pthread_mutex_lock(&this->read_mutex);
		return result;
	}

	int Serial::SerialImpl::readUnlock (){
		int result = pthread_mutex_unlock(&this->read_mutex);
		return result;
	}

	int Serial::SerialImpl::writeLock (){
		int result = pthread_mutex_lock(&this->write_mutex);
		return result;
	}

	int Serial::SerialImpl::writeUnlock (){
		int result = pthread_mutex_unlock(&this->write_mutex);
		return result;
	}
}
#endif // !defined(_WIN32)
