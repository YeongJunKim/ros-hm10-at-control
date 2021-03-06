/*
 * Serial.cpp
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */
#include "Serial.h"
#include <iostream>
//#include <fstream>
#include <termios.h>
#include <fcntl.h>
using namespace std;

int open_serial(char *dev_name, int baud, int vtime, int vmin)
{
    int fd;
    struct termios newtio;

    fd = open(dev_name, O_RDWR | O_NOCTTY);
    if(fd == 0)
    {
    	cout << "fail" << endl;
        printf("fail open");
        return -1;
    }
    else
    {
        cout << "open good" <<endl;
    }
    

    // port configure
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR;    // no-parity
    newtio.c_oflag = 0;

    newtio.c_cflag = CS8 | CLOCAL | CREAD;  // no-rts & no-cts

    switch(baud)
    {
		case 500000 : newtio.c_cflag |= B500000; break;
		case 250000 : newtio.c_cflag |= B230400; break;
		case 115200 : newtio.c_cflag |= B115200; break;
		case 57600  : newtio.c_cflag |= B57600; break;
		case 9600   : newtio.c_cflag |= B9600; break;
		default     : newtio.c_cflag |= B115200; break;
    }

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = vtime;     // timeout 0.1s
    newtio.c_cc[VMIN] = vmin;       // wait

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    return fd;
}
void close_serial(int fd)
{
    close(fd);
}
void sendMessage(char* text, int cnt, int fd)
{
    char buf[128]={0};
   for(int i = 0; i < cnt; i ++)
    {
        memset(buf,*(text+i),1);
        write(fd, buf, 1);
    }
}



