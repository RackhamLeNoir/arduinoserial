#ifndef _SERIAL_
#define _SERIAL_

#define ARDUINO_WAIT_TIME 500

class Serial
{
    public:
        //Initialize Serial communication with the given COM port
		//baudrate among 300 1200 2400 4800 9600 14400 19200 38400 57600 115200
        __declspec(dllexport) Serial(const char *portName, int baudrate = 57600);

        //Close the connection
        //NOTA: for some reason you can't connect again before exiting
        //the program and running it again
        __declspec(dllexport) virtual ~Serial();

        //Read data in a buffer, if nbChar is greater than the
        //maximum number of bytes available, it will return only the
        //bytes available. The function return -1 when nothing could
        //be read, the number of bytes actually read.
        __declspec(dllexport) virtual int ReadData(void *buffer, unsigned int nbChar)=0;

        //Writes data from a buffer through the Serial connection
        //return true on success.
        __declspec(dllexport) virtual bool WriteData(void *buffer, unsigned int nbChar)=0;

        //Check if we are actually connected
		__declspec(dllexport) bool IsConnected() { return _connected; }

    protected:
        //Connection status
        bool _connected;
};

#endif
