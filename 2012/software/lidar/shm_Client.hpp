#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <stdlib.h>
#include "NAV200.hpp"


class SHM_CLIENT{

public:
    SHM_CLIENT(int key, int size);
    bool connect();
    float* shmRead();

private:
    key_t key;
    int shmID;
    float* shm;
    int shmSize;
};
