#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

#define max(a,b) ( (a) > (b) ? (a):(b))
#define min(a,b) ( (a) < (b) ? (a):(b))

int main(int argc, char** argv) {

    if (argc != 3) {
        fprintf(stderr, "Usage: %s devicefile value\nset environment variable PWM_SAT to print debug output\n", argv[0]); 
        exit(1);
    }

    int retval;
    int fd = open(argv[1], O_RDWR|O_NOCTTY|O_NDELAY);
    fcntl(fd, F_SETFL, FNDELAY);
    if (fd == -1) {
        perror("open serial device");
        exit(1);
    }

    struct termios my_termios;

    retval = tcgetattr( fd, &my_termios );
    if (retval == -1) {
        perror("getattr");
        exit(1);
    }


    retval = cfsetispeed(&my_termios, B38400);
    retval = cfsetospeed(&my_termios, B38400);
    if (retval == -1) {
        perror("setspeed");
        exit(1);
    }
    cfmakeraw(&my_termios);

    my_termios.c_iflag &= ~(IXON | IXOFF | IXANY);
    my_termios.c_iflag |= IGNBRK;
    my_termios.c_cflag &= ~(PARENB|CSTOPB|CSIZE|CREAD|CRTSCTS);
    my_termios.c_cflag |= CS8;
    my_termios.c_cflag |= HUPCL;
    my_termios.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    my_termios.c_cc[VMIN]=1;
    my_termios.c_cc[VTIME] = 5;

    my_termios.c_cflag |= CRTSCTS;
    retval = tcsetattr( fd, TCSANOW, &my_termios );

    my_termios.c_cflag &= ~(PARENB|CSTOPB|CSIZE|CREAD|CRTSCTS);
    retval = tcsetattr( fd, TCSANOW, &my_termios );
    if (retval == -1) {
        perror("setattr");
        exit(1);
    }

    int val = min(255, max(0, atoi(argv[2])));
    if (getenv("PWM_SAT") != NULL) {
        printf("saturated value: %d\n", val);
    }

    unsigned char v = (unsigned char) val;
    printf("value: %d\n", v);
    if (write(fd, &v, 1) != 1) {
        perror("write");
        exit(1);
    }

    close(fd);

    return 0;
}
