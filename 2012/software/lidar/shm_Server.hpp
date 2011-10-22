#pragma once

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <stdlib.h>
#include "NAV200.hpp"
#include "shm_id.h"

#define SHMSZ sizeof(NAV200::Point)*(NAV200::Num_Points + 2)


class SHM_SERVER{
public:
    SHM_SERVER(int key);
    ~SHM_SERVER();

    bool setup(int size);
    float* getMem();
    bool shmWrite(float* data);

private:
    key_t key;
    int shmID;
    float* shm;
    int shmSize;

};
