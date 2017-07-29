#ifndef MAESTRO_H
#define MAESTRO_H

#ifdef RASP
#include <termios.h>
#endif

class Maestro
{
    private:
        const char * device = "/dev/ttyAMA0";  // Linux
        int fd;

        #ifdef RASP
        struct termios options;
        #endif
    public:
        Maestro();
        ~Maestro();

        int getError();
        int getPosition(unsigned char channel);
        int setTarget(unsigned char channel, unsigned short target);
};

#endif // MAESTRO_H
