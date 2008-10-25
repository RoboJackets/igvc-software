#include <stdio.h>

#define MAX_READINGS 769
#define  B115200  0010002

typedef struct urg_laser_readings {
    unsigned short Readings[MAX_READINGS];
} urg_laser_readings_t;

class urg_laser {
public:
    urg_laser();
    ~urg_laser();

    int Open(const char * PortName, int use_serial, int baud);
    int Close();
    int ChangeBaud(int curr_baud, int new_baud, int timeout);
    int ReadUntil(int fd, unsigned char *buf, int len, int timeout);

    int ReadUntil_nthOccurence(int file, int n, char c);

    bool PortOpen();

    //int GetReadings(urg_laser_readings_t * readings);
    int GetReadings     (urg_laser_readings_t * readings, int min_i, int max_i);
    int GetIDInfo       ();
    float GetMaxRange   ();
    int GetSCIPVersion  ();

private:
    FILE * laser_port;
    int SCIP_Version;

};
