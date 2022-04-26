#include "SerialMac.hpp"

#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


#include <iostream>
using namespace std;

SerialMac::SerialMac(const char *portName, int baudrate)
:Serial(portName)
{
    //Try to connect to the given port throuh CreateFile
    _hSerial = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    //Check if the connection was successfull
    if(_hSerial < 0)
    {
		char buffer[256];
		sprintf(buffer, "Open error on port %s %d %s\n", portName, errno, strerror(errno));
		throw buffer;
    }
    
    struct termios serialParams;
    
    //Try to get the current
    if (tcgetattr(_hSerial, &serialParams) < 0)
    {
		throw "Cannot get COM port properties";

    }
    
    tcflush(_hSerial, TCIFLUSH);
    
    serialParams.c_cflag = CS8 | CREAD;

    //Define serial connection parameters for the arduino board
    switch(baudrate)
    {        
        case 300:
            cfsetispeed(&serialParams, B300);
            cfsetospeed(&serialParams, B300);
            serialParams.c_cflag |= B300; 
            break;
        case 1200:
            cfsetispeed(&serialParams, B1200);
            cfsetospeed(&serialParams, B1200);
            serialParams.c_cflag |= B1200; 
            break;
        case 2400:
            cfsetispeed(&serialParams, B2400);
            cfsetospeed(&serialParams, B2400);
            serialParams.c_cflag |= B2400; 
            break;
        case 4800:
            cfsetispeed(&serialParams, B4800);
            cfsetospeed(&serialParams, B4800);
            serialParams.c_cflag |= B4800; 
            break;
        case 9600:
            cfsetispeed(&serialParams, B9600);
            cfsetospeed(&serialParams, B9600);
            serialParams.c_cflag |= B9600; 
            break;
        case 14400:
            cfsetispeed(&serialParams, B14400);
            cfsetospeed(&serialParams, B14400);
            serialParams.c_cflag |= B14400; 
            break;
        case 19200:
            cfsetispeed(&serialParams, B19200);
            cfsetospeed(&serialParams, B19200);
            serialParams.c_cflag |= B19200; 
            break;
        case 38400:
            cfsetispeed(&serialParams, B38400);
            cfsetospeed(&serialParams, B38400);
            serialParams.c_cflag |= B38400; 
            break;
        case 57600:
            cfsetispeed(&serialParams, B57600);
            cfsetospeed(&serialParams, B57600);
            serialParams.c_cflag |= B57600; 
            break;
        case 115200:
            cfsetispeed(&serialParams, B115200);
            cfsetospeed(&serialParams, B115200);
            serialParams.c_cflag |= B115200; 
            break;
        default:
            cfsetispeed(&serialParams, B57600);
            cfsetospeed(&serialParams, B57600);
            serialParams.c_cflag |= B57600; 
            break;
    }
    //No parity
    serialParams.c_cflag &= ~PARENB;
    //One stop bit
    serialParams.c_cflag &= ~CSTOPB;
    
    if (tcsetattr(_hSerial, TCSANOW, &serialParams) < 0)
    {
        char buffer[256];
        sprintf(buffer, "ERROR: Could not set Serial Port parameters port:%s error:%d %s\n", portName, errno, strerror(errno));
        cerr << buffer << endl;
        throw buffer;
    }
    else
    {
        //If everything went fine we're connected
        _connected = true;
        //We wait 2s as the arduino board will be reseting
        //sleep(ARDUINO_WAIT_TIME);
        //SDL_Delay(ARDUINO_WAIT_TIME);
    }
}

SerialMac::~SerialMac()
{
    //Check if we are connected before trying to disconnect
    if(_connected)
    {
        //We're no longer connected
        _connected = false;
        //Close the serial handler
        close(_hSerial);
    }
}

int SerialMac::ReadData(void *buffer, unsigned int nbChar)
{
    //Number of bytes we'll really ask to read
    int r = 0;
    
    if ((r = read(_hSerial, buffer, size_t(nbChar))) < 0)
    {
        if (errno == EAGAIN)
            return 0;
        else
            cerr << "read error " << errno << " " << strerror(errno) << endl;
    }

    return r;
}


bool SerialMac::WriteData(void *buffer, unsigned int nbChar)
{
    int r = 0;

    //Try to write the buffer on the Serial port
    if(write(_hSerial, buffer, nbChar) < 0)
    {
        cerr << "write error " << errno << " " << strerror(errno) << endl;
        return false;
    }
    return true;
}
