#ifndef SERIAL_H
#define SERIAL_H

#include <limits>
#include <vector>
#include <string>
#include <cstring>
#include <sstream>
#include "v8stdint.h"

namespace serial {

	/*!
	* Enumeration defines the possible bytesizes for the serial port.
	*/
	typedef enum {
		fivebits = 5,
		sixbits = 6,
		sevenbits = 7,
		eightbits = 8
	} bytesize_t;

	/*!
	* Enumeration defines the possible parity types for the serial port.
	*/
	typedef enum {
		parity_none = 0,
		parity_odd = 1,
		parity_even = 2,
		parity_mark = 3,
		parity_space = 4
	} parity_t;

	/*!
	* Enumeration defines the possible stopbit types for the serial port.
	*/
	typedef enum {
		stopbits_one = 1,
		stopbits_two = 2,
		stopbits_one_point_five
	} stopbits_t;

	/*!
	* Enumeration defines the possible flowcontrol types for the serial port.
	*/
	typedef enum {
		flowcontrol_none = 0,
		flowcontrol_software,
		flowcontrol_hardware
	} flowcontrol_t;

	/*!
	* Structure for setting the timeout of the serial port, times are
	* in milliseconds.
	*
	* In order to disable the interbyte timeout, set it to Timeout::max().
	*/
	struct Timeout {
#ifdef max
# undef max
#endif
		static uint32_t max() {return std::numeric_limits<uint32_t>::max();}
		/*!
		* Convenience function to generate Timeout structs using a
		* single absolute timeout.
		*
		* \param timeout A long that defines the time in milliseconds until a
		* timeout occurs after a call to read or write is made.
		*
		* \return Timeout struct that represents this simple timeout provided.
		*/
		static Timeout simpleTimeout(uint32_t timeout) {
			return Timeout(max(), timeout, 0, timeout, 0);
		}

		/*! Number of milliseconds between bytes received to timeout on. */
		uint32_t inter_byte_timeout;
		/*! A constant number of milliseconds to wait after calling read. */
		uint32_t read_timeout_constant;
		/*! A multiplier against the number of requested bytes to wait after
		*  calling read.
		*/
		uint32_t read_timeout_multiplier;
		/*! A constant number of milliseconds to wait after calling write. */
		uint32_t write_timeout_constant;
		/*! A multiplier against the number of requested bytes to wait after
		*  calling write.
		*/
		uint32_t write_timeout_multiplier;

		explicit Timeout (uint32_t inter_byte_timeout_=0,
			uint32_t read_timeout_constant_=0,
			uint32_t read_timeout_multiplier_=0,
			uint32_t write_timeout_constant_=0,
			uint32_t write_timeout_multiplier_=0)
			: inter_byte_timeout(inter_byte_timeout_),
			read_timeout_constant(read_timeout_constant_),
			read_timeout_multiplier(read_timeout_multiplier_),
			write_timeout_constant(write_timeout_constant_),
			write_timeout_multiplier(write_timeout_multiplier_)
		{}
	};

	/*!
	* Class that provides a portable serial port interface.
	*/
	class Serial {
	public:
		/*!
		* Creates a Serial object and opens the port if a port is specified,
		* otherwise it remains closed until serial::Serial::open is called.
		*
		* \param port A std::string containing the address of the serial port,
		*        which would be something like 'COM1' on Windows and '/dev/ttyS0'
		*        on Linux.
		*
		* \param baudrate An unsigned 32-bit integer that represents the baudrate
		*
		* \param timeout A serial::Timeout struct that defines the timeout
		* conditions for the serial port. \see serial::Timeout
		*
		* \param bytesize Size of each byte in the serial transmission of data,
		* default is eightbits, possible values are: fivebits, sixbits, sevenbits,
		* eightbits
		*
		* \param parity Method of parity, default is parity_none, possible values
		* are: parity_none, parity_odd, parity_even
		*
		* \param stopbits Number of stop bits used, default is stopbits_one,
		* possible values are: stopbits_one, stopbits_one_point_five, stopbits_two
		*
		* \param flowcontrol Type of flowcontrol used, default is
		* flowcontrol_none, possible values are: flowcontrol_none,
		* flowcontrol_software, flowcontrol_hardware
		*
		* \throw serial::PortNotOpenedException
		* \throw serial::IOException
		* \throw std::invalid_argument
		*/
        explicit Serial (const std::string &port = "",
			uint32_t baudrate = 9600,
			Timeout timeout = Timeout(),
			bytesize_t bytesize = eightbits,
			parity_t parity = parity_none,
			stopbits_t stopbits = stopbits_one,
			flowcontrol_t flowcontrol = flowcontrol_none);

		/*! Destructor */
		virtual ~Serial ();

		/*!
		* Opens the serial port as long as the port is set and the port isn't
		* already open.
		*
		* If the port is provided to the constructor then an explicit call to open
		* is not needed.
		*
		* \see Serial::Serial
		* \return Returns true if the port is open, false otherwise.
		*/
		bool open ();

		/*! Gets the open status of the serial port.
		*
		* \return Returns true if the port is open, false otherwise.
		*/
		bool isOpen () const;

		/*! Closes the serial port. */
		void close ();

		/*! Return the number of characters in the buffer. */
		size_t available ();

		/*! Block until there is serial data to read or read_timeout_constant
		* number of milliseconds have elapsed. The return value is true when
		* the function exits with the port in a readable state, false otherwise
		* (due to timeout or select interruption). */
		bool waitReadable ();

		/*! Block for a period of time corresponding to the transmission time of
		* count characters at present serial settings. This may be used in con-
		* junction with waitReadable to read larger blocks of data from the
		* port. */
		void waitByteTimes (size_t count);


		int waitfordata(size_t data_count, uint32_t timeout, size_t * returned_size);

		/*! Read a given amount of bytes from the serial port into a given buffer.
		*
		* The read function will return in one of three cases:
		*  * The number of requested bytes was read.
		*    * In this case the number of bytes requested will match the size_t
		*      returned by read.
		*  * A timeout occurred, in this case the number of bytes read will not
		*    match the amount requested, but no exception will be thrown.  One of
		*    two possible timeouts occurred:
		*    * The inter byte timeout expired, this means that number of
		*      milliseconds elapsed between receiving bytes from the serial port
		*      exceeded the inter byte timeout.
		*    * The total timeout expired, which is calculated by multiplying the
		*      read timeout multiplier by the number of requested bytes and then
		*      added to the read timeout constant.  If that total number of
		*      milliseconds elapses after the initial call to read a timeout will
		*      occur.
		*  * An exception occurred, in this case an actual exception will be thrown.
		*
		* \param buffer An uint8_t array of at least the requested size.
		* \param size A size_t defining how many bytes to be read.
		*
		* \return A size_t representing the number of bytes read as a result of the
		*         call to read.
		*
		*/
		size_t read (uint8_t *buffer, size_t size);

		/*! Read a given amount of bytes from the serial port into a give buffer.
		*
		* \param buffer A reference to a std::vector of uint8_t.
		* \param size A size_t defining how many bytes to be read.
		*
		* \return A size_t representing the number of bytes read as a result of the
		*         call to read.
		*
		*/
		size_t read (std::vector<uint8_t> &buffer, size_t size = 1);

		/*! Read a given amount of bytes from the serial port into a give buffer.
		*
		* \param buffer A reference to a std::string.
		* \param size A size_t defining how many bytes to be read.
		*
		* \return A size_t representing the number of bytes read as a result of the
		*         call to read.
		*
		*/
		size_t read (std::string &buffer, size_t size = 1);

		/*! Read a given amount of bytes from the serial port and return a string
		*  containing the data.
		*
		* \param size A size_t defining how many bytes to be read.
		*
		* \return A std::string containing the data read from the port.
		*
		*/
		std::string read (size_t size = 1);

		/*! Reads in a line or until a given delimiter has been processed.
		*
		* Reads from the serial port until a single line has been read.
		*
		* \param buffer A std::string reference used to store the data.
		* \param size A maximum length of a line, defaults to 65536 (2^16)
		* \param eol A string to match against for the EOL.
		*
		* \return A size_t representing the number of bytes read.
		*
		*/
		size_t readline (std::string &buffer, size_t size = 65536, std::string eol = "\n");

		/*! Reads in a line or until a given delimiter has been processed.
		*
		* Reads from the serial port until a single line has been read.
		*
		* \param size A maximum length of a line, defaults to 65536 (2^16)
		* \param eol A string to match against for the EOL.
		*
		* \return A std::string containing the line.
		*
		*/
		std::string readline (size_t size = 65536, std::string eol = "\n");

		/*! Reads in multiple lines until the serial port times out.
		*
		* This requires a timeout > 0 before it can be run. It will read until a
		* timeout occurs and return a list of strings.
		*
		* \param size A maximum length of combined lines, defaults to 65536 (2^16)
		*
		* \param eol A string to match against for the EOL.
		*
		* \return A vector<string> containing the lines.
		*
		*/
		std::vector<std::string> readlines (size_t size = 65536, std::string eol = "\n");

		/*! Write a string to the serial port.
		*
		* \param data A const reference containing the data to be written
		* to the serial port.
		*
		* \param size A size_t that indicates how many bytes should be written from
		* the given data buffer.
		*
		* \return A size_t representing the number of bytes actually written to
		* the serial port.
		*
		* \throw serial::PortNotOpenedException
		* \throw serial::SerialException
		* \throw serial::IOException
		*/
		size_t write (const uint8_t *data, size_t size);

		/*! Write a string to the serial port.
		*
		* \param data A const reference containing the data to be written
		* to the serial port.
		*
		* \return A size_t representing the number of bytes actually written to
		* the serial port.
		*
		*/
		size_t write (const std::vector<uint8_t> &data);

		/*! Write a string to the serial port.
		*
		* \param data A const reference containing the data to be written
		* to the serial port.
		*
		* \return A size_t representing the number of bytes actually written to
		* the serial port.
		*
		*/
		size_t write (const std::string &data);

		/*! Sets the serial port identifier.
		*
		* \param port A const std::string reference containing the address of the
		* serial port, which would be something like 'COM1' on Windows and
		* '/dev/ttyS0' on Linux.
		*
		* \throw std::invalid_argument
		*/
		void setPort (const std::string &port);

		/*! Gets the serial port identifier.
		*
		* \see Serial::setPort
		*
		* \throw std::invalid_argument
		*/
		std::string getPort () const;

		/*! Sets the timeout for reads and writes using the Timeout struct.
		*
		* There are two timeout conditions described here:
		*  * The inter byte timeout:
		*    * The inter_byte_timeout component of serial::Timeout defines the
		*      maximum amount of time, in milliseconds, between receiving bytes on
		*      the serial port that can pass before a timeout occurs.  Setting this
		*      to zero will prevent inter byte timeouts from occurring.
		*  * Total time timeout:
		*    * The constant and multiplier component of this timeout condition,
		*      for both read and write, are defined in serial::Timeout.  This
		*      timeout occurs if the total time since the read or write call was
		*      made exceeds the specified time in milliseconds.
		*    * The limit is defined by multiplying the multiplier component by the
		*      number of requested bytes and adding that product to the constant
		*      component.  In this way if you want a read call, for example, to
		*      timeout after exactly one second regardless of the number of bytes
		*      you asked for then set the read_timeout_constant component of
		*      serial::Timeout to 1000 and the read_timeout_multiplier to zero.
		*      This timeout condition can be used in conjunction with the inter
		*      byte timeout condition with out any problems, timeout will simply
		*      occur when one of the two timeout conditions is met.  This allows
		*      users to have maximum control over the trade-off between
		*      responsiveness and efficiency.
		*
		* Read and write functions will return in one of three cases.  When the
		* reading or writing is complete, when a timeout occurs, or when an
		* exception occurs.
		*
		* A timeout of 0 enables non-blocking mode.
		*
		* \param timeout A serial::Timeout struct containing the inter byte
		* timeout, and the read and write timeout constants and multipliers.
		*
		* \see serial::Timeout
		*/
		void setTimeout (Timeout &timeout);

		/*! Sets the timeout for reads and writes. */
		void setTimeout (uint32_t inter_byte_timeout, uint32_t read_timeout_constant,
			uint32_t read_timeout_multiplier, uint32_t write_timeout_constant,
			uint32_t write_timeout_multiplier)
		{
			Timeout timeout(inter_byte_timeout, read_timeout_constant,
				read_timeout_multiplier, write_timeout_constant,
				write_timeout_multiplier);
			return setTimeout(timeout);
		}

		/*! Gets the timeout for reads in seconds.
		*
		* \return A Timeout struct containing the inter_byte_timeout, and read
		* and write timeout constants and multipliers.
		*
		* \see Serial::setTimeout
		*/
		Timeout getTimeout () const;

		/*! Sets the baudrate for the serial port.
		*
		* Possible baudrates depends on the system but some safe baudrates include:
		* 110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000,
		* 57600, 115200
		* Some other baudrates that are supported by some comports:
		* 128000, 153600, 230400, 256000, 460800, 921600
		*
		* \param baudrate An integer that sets the baud rate for the serial port.
		*
		*/
		bool setBaudrate (uint32_t baudrate);

		/*! Gets the baudrate for the serial port.
		*
		* \return An integer that sets the baud rate for the serial port.
		*
		* \see Serial::setBaudrate
		*
		* \
		*/
		uint32_t getBaudrate () const;

		/*! Sets the bytesize for the serial port.
		*
		* \param bytesize Size of each byte in the serial transmission of data,
		* default is eightbits, possible values are: fivebits, sixbits, sevenbits,
		* eightbits
		*
		* \
		*/
		bool setBytesize (bytesize_t bytesize);

		/*! Gets the bytesize for the serial port.
		*
		* \see Serial::setBytesize
		*
		* \
		*/
		bytesize_t getBytesize () const;

		/*! Sets the parity for the serial port.
		*
		* \param parity Method of parity, default is parity_none, possible values
		* are: parity_none, parity_odd, parity_even
		*
		* \
		*/
		bool setParity (parity_t parity);

		/*! Gets the parity for the serial port.
		*
		* \see Serial::setParity
		*
		* \
		*/
		parity_t getParity () const;

		/*! Sets the stopbits for the serial port.
		*
		* \param stopbits Number of stop bits used, default is stopbits_one,
		* possible values are: stopbits_one, stopbits_one_point_five, stopbits_two
		*
		* \
		*/
		bool setStopbits (stopbits_t stopbits);

		/*! Gets the stopbits for the serial port.
		*
		* \see Serial::setStopbits
		*
		* \
		*/
		stopbits_t getStopbits () const;

		/*! Sets the flow control for the serial port.
		*
		* \param flowcontrol Type of flowcontrol used, default is flowcontrol_none,
		* possible values are: flowcontrol_none, flowcontrol_software,
		* flowcontrol_hardware
		*
		* \
		*/
		bool setFlowcontrol (flowcontrol_t flowcontrol);

		/*! Gets the flow control for the serial port.
		*
		* \see Serial::setFlowcontrol
		*
		* \
		*/
		flowcontrol_t getFlowcontrol () const;

		/*! Flush the input and output buffers */
		void flush ();

		/*! Flush only the input buffer */
		void flushInput ();

		/*! Flush only the output buffer */
		void flushOutput ();

		/*! Sends the RS-232 break signal.  See tcsendbreak(3). */
		void sendBreak (int duration);

		/*! Set the break condition to a given level.  Defaults to true. */
		bool setBreak (bool level = true);

		/*! Set the RTS handshaking line to the given level.  Defaults to true. */
		bool setRTS (bool level = true);

		/*! Set the DTR handshaking line to the given level.  Defaults to true. */
		bool setDTR (bool level = true);

		/*!
		* Blocks until CTS, DSR, RI, CD changes or something interrupts it.
		*
		* Can throw an exception if an error occurs while waiting.
		* You can check the status of CTS, DSR, RI, and CD once this returns.
		* Uses TIOCMIWAIT via ioctl if available (mostly only on Linux) with a
		* resolution of less than +-1ms and as good as +-0.2ms.  Otherwise a
		* polling method is used which can give +-2ms.
		*
		* \return Returns true if one of the lines changed, false if something else
		* occurred.
		*
		*/
		bool waitForChange ();

		/*! Returns the current status of the CTS line. */
		bool getCTS ();

		/*! Returns the current status of the DSR line. */
		bool getDSR ();

		/*! Returns the current status of the RI line. */
		bool getRI ();

		/*! Returns the current status of the CD line. */
		bool getCD ();

		/*! Returns the singal byte time. */
		uint32_t getByteTime();

	private:
		// Disable copy constructors
        Serial(const Serial&);
		Serial& operator=(const Serial&);

		// Pimpl idiom, d_pointer
		class SerialImpl;
		SerialImpl *pimpl_;

		// Scoped Lock Classes
		class ScopedReadLock;
		class ScopedWriteLock;

		// Read common function
		size_t read_ (uint8_t *buffer, size_t size);
		// Write common function
		size_t write_ (const uint8_t *data, size_t length);
	};


	/*!
	* Structure that describes a serial device.
	*/
	struct PortInfo {

		/*! Address of the serial port (this can be passed to the constructor of Serial). */
		std::string port;

		/*! Human readable description of serial device if available. */
		std::string description;

		/*! Hardware ID (e.g. VID:PID of USB serial devices) or "n/a" if not available. */
		std::string hardware_id;

	};

	/* Lists the serial ports available on the system
	*
	* Returns a vector of available serial ports, each represented
	* by a serial::PortInfo data structure:
	*
	* \return vector of serial::PortInfo.
	*/
	//std::vector<PortInfo> list_ports();

} // namespace serial

#endif
