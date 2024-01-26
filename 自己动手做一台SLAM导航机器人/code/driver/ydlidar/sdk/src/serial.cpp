#include <algorithm>

#if !defined(_WIN32) && !defined(__OpenBSD__) && !defined(__FreeBSD__)
# include <alloca.h>
#endif

#if defined (__MINGW32__)
# define alloca __builtin_alloca
#endif

#include "serial.h"
#include "common.h"

namespace serial {

	using std::min;
	using std::numeric_limits;
	using std::vector;
	using std::size_t;
	using std::string;

	using serial::Serial;
	using serial::bytesize_t;
	using serial::parity_t;
	using serial::stopbits_t;
	using serial::flowcontrol_t;

	class Serial::ScopedReadLock {
	public:
        explicit ScopedReadLock(SerialImpl *pimpl) : pimpl_(pimpl) {
			this->pimpl_->readLock();
		}
		~ScopedReadLock() {
			this->pimpl_->readUnlock();
		}
	private:
		// Disable copy constructors
		ScopedReadLock(const ScopedReadLock&);
		const ScopedReadLock& operator=(ScopedReadLock);

		SerialImpl *pimpl_;
	};

	class Serial::ScopedWriteLock {
	public:
        explicit ScopedWriteLock(SerialImpl *pimpl) : pimpl_(pimpl) {
			this->pimpl_->writeLock();
		}
		~ScopedWriteLock() {
			this->pimpl_->writeUnlock();
		}
	private:
		// Disable copy constructors
		ScopedWriteLock(const ScopedWriteLock&);
		const ScopedWriteLock& operator=(ScopedWriteLock);
		SerialImpl *pimpl_;
	};

	Serial::Serial (const string &port, uint32_t baudrate, serial::Timeout timeout,
		bytesize_t bytesize, parity_t parity, stopbits_t stopbits,
		flowcontrol_t flowcontrol)
		: pimpl_(new SerialImpl (port, baudrate, bytesize, parity,
		stopbits, flowcontrol)) {
		pimpl_->setTimeout(timeout);
	}

	Serial::~Serial () {
		delete pimpl_;
	}

	bool Serial::open () {
		return pimpl_->open ();
	}

	void Serial::close () {
		pimpl_->close ();
	}

	bool Serial::isOpen () const {
		return pimpl_->isOpen ();
	}

	size_t Serial::available () {
		return pimpl_->available ();
	}

	bool Serial::waitReadable () {
		serial::Timeout timeout(pimpl_->getTimeout ());
		return pimpl_->waitReadable(timeout.read_timeout_constant);
	}

	void Serial::waitByteTimes (size_t count) {
		pimpl_->waitByteTimes(count);
	}

	int Serial::waitfordata(size_t data_count, uint32_t timeout, size_t * returned_size) {
		return pimpl_->waitfordata(data_count, timeout, returned_size);
	}

	size_t Serial::read_ (uint8_t *buffer, size_t size) {
		return this->pimpl_->read (buffer, size);
	}

	size_t Serial::read (uint8_t *buffer, size_t size) {
		ScopedReadLock lock(this->pimpl_);
		return this->pimpl_->read (buffer, size);
	}

	size_t Serial::read (std::vector<uint8_t> &buffer, size_t size) {
		ScopedReadLock lock(this->pimpl_);
		uint8_t *buffer_ = new uint8_t[size];
		size_t bytes_read = this->pimpl_->read (buffer_, size);
		buffer.insert (buffer.end (), buffer_, buffer_+bytes_read);
		delete[] buffer_;
		return bytes_read;
	}

	size_t Serial::read (std::string &buffer, size_t size) {
		ScopedReadLock lock(this->pimpl_);
		uint8_t *buffer_ = new uint8_t[size];
		size_t bytes_read = this->pimpl_->read (buffer_, size);
		buffer.append (reinterpret_cast<const char*>(buffer_), bytes_read);
		delete[] buffer_;
		return bytes_read;
	}

	string Serial::read (size_t size) {
		std::string buffer;
		this->read (buffer, size);
		return buffer;
	}

	size_t Serial::readline (string &buffer, size_t size, string eol) {
		ScopedReadLock lock(this->pimpl_);
		size_t eol_len = eol.length ();
		uint8_t *buffer_ = static_cast<uint8_t*> (alloca (size * sizeof (uint8_t)));
		size_t read_so_far = 0;
		while (true) {
			size_t bytes_read = this->read_ (buffer_ + read_so_far, 1);
			read_so_far += bytes_read;
			if (bytes_read == 0) {
				break; // Timeout occured on reading 1 byte
			}
			if (string (reinterpret_cast<const char*> (buffer_ + read_so_far - eol_len), eol_len) == eol) {
					break; // EOL found
			}
			if (read_so_far == size) {
				break; // Reached the maximum read length
			}
		}
		buffer.append(reinterpret_cast<const char*> (buffer_), read_so_far);
		return read_so_far;
	}

	string Serial::readline (size_t size, string eol) {
		std::string buffer;
		this->readline (buffer, size, eol);
		return buffer;
	}

	vector<string> Serial::readlines (size_t size, string eol) {
		ScopedReadLock lock(this->pimpl_);
		std::vector<std::string> lines;
		size_t eol_len = eol.length ();
		uint8_t *buffer_ = static_cast<uint8_t*> (alloca (size * sizeof (uint8_t)));
		size_t read_so_far = 0;
		size_t start_of_line = 0;
		while (read_so_far < size) {
			size_t bytes_read = this->read_ (buffer_+read_so_far, 1);
			read_so_far += bytes_read;
			if (bytes_read == 0) {
				if (start_of_line != read_so_far) {
					lines.push_back ( string (reinterpret_cast<const char*> (buffer_ + start_of_line), read_so_far - start_of_line));
				}
				break; // Timeout occured on reading 1 byte
			}
			if (string (reinterpret_cast<const char*>
				(buffer_ + read_so_far - eol_len), eol_len) == eol) {
					// EOL found
					lines.push_back( string(reinterpret_cast<const char*> (buffer_ + start_of_line), read_so_far - start_of_line));
					start_of_line = read_so_far;
			}
			if (read_so_far == size) {
				if (start_of_line != read_so_far) {
					lines.push_back( string(reinterpret_cast<const char*> (buffer_ + start_of_line), read_so_far - start_of_line));
				}
				break; // Reached the maximum read length
			}
		}
		return lines;
	}

	size_t Serial::write (const string &data) {
		ScopedWriteLock lock(this->pimpl_);
		return this->write_ (reinterpret_cast<const uint8_t*>(data.c_str()), data.length());
	}

	size_t Serial::write (const std::vector<uint8_t> &data) {
		ScopedWriteLock lock(this->pimpl_);
		return this->write_ (&data[0], data.size());
	}

	size_t Serial::write (const uint8_t *data, size_t size) {
		ScopedWriteLock lock(this->pimpl_);
		return this->write_(data, size);
	}

	size_t Serial::write_ (const uint8_t *data, size_t length) {
		return pimpl_->write (data, length);
	}

	void Serial::setPort (const string &port) {
		ScopedReadLock rlock(this->pimpl_);
		ScopedWriteLock wlock(this->pimpl_);
		bool was_open = pimpl_->isOpen ();
		if (was_open) close();
		pimpl_->setPort (port);
		if (was_open) open ();
	}

	string Serial::getPort () const {
		return pimpl_->getPort ();
	}

	void Serial::setTimeout (serial::Timeout &timeout) {
		pimpl_->setTimeout (timeout);
	}

	serial::Timeout Serial::getTimeout () const {
			return pimpl_->getTimeout ();
	}

	bool Serial::setBaudrate (uint32_t baudrate) {
		return pimpl_->setBaudrate (baudrate);
	}

	uint32_t Serial::getBaudrate () const {
		return uint32_t(pimpl_->getBaudrate ());
	}

	bool Serial::setBytesize (bytesize_t bytesize){
		return pimpl_->setBytesize (bytesize);
	}

	bytesize_t Serial::getBytesize () const{
		return pimpl_->getBytesize ();
	}

	bool Serial::setParity (parity_t parity){
		return pimpl_->setParity (parity);
	}

	parity_t Serial::getParity () const{
		return pimpl_->getParity ();
	}

	bool Serial::setStopbits (stopbits_t stopbits){
		return pimpl_->setStopbits (stopbits);
	}

	stopbits_t Serial::getStopbits () const{
		return pimpl_->getStopbits ();
	}

	bool Serial::setFlowcontrol (flowcontrol_t flowcontrol){
		return pimpl_->setFlowcontrol (flowcontrol);
	}

	flowcontrol_t Serial::getFlowcontrol () const{
		return pimpl_->getFlowcontrol ();
	}

	void Serial::flush (){
		ScopedReadLock rlock(this->pimpl_);
		ScopedWriteLock wlock(this->pimpl_);
		pimpl_->flush ();
	}

	void Serial::flushInput (){
		ScopedReadLock lock(this->pimpl_);
		pimpl_->flushInput ();
	}

	void Serial::flushOutput (){
		ScopedWriteLock lock(this->pimpl_);
		pimpl_->flushOutput ();
	}

	void Serial::sendBreak (int duration){
		pimpl_->sendBreak (duration);
	}

	bool Serial::setBreak (bool level){
		return pimpl_->setBreak (level);
	}

	bool Serial::setRTS (bool level){
		return pimpl_->setRTS (level);
	}

	bool Serial::setDTR (bool level){
		return pimpl_->setDTR (level);
	}

	bool Serial::waitForChange(){
		return pimpl_->waitForChange();
	}

	bool Serial::getCTS (){
		return pimpl_->getCTS ();
	}

	bool Serial::getDSR (){
		return pimpl_->getDSR ();
	}

	bool Serial::getRI (){
		return pimpl_->getRI ();
	}

	bool Serial::getCD (){
		return pimpl_->getCD ();
	}

	uint32_t Serial::getByteTime(){
		return pimpl_->getByteTime();
	}
}
