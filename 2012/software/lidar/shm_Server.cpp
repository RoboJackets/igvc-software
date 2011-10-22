#include "shm_Server.hpp"
#include "shm_id.h"
#include "NAV200.hpp"

using namespace std;

SHM_SERVER::SHM_SERVER(int key):key(key){

    this->shmID = -1;
}

SHM_SERVER::~SHM_SERVER(){
    delete shm;
}

bool SHM_SERVER::setup(int size){

    shmSize = size;

    if((shmID = shmget(this->key, size, IPC_CREAT | 0666)) < 0){

	printf("%d",size);
        perror("Error getting shared memory.\n");
        return false;
    }

    if((shm = (float*)shmat(this->shmID, NULL, 0)) == NULL){
        perror("Error attaching to shared memory.\n");
        return false;
    }

    return true;


}

bool SHM_SERVER::shmWrite(float* data){

    for (int i = 0; i < NAV200::Num_Points; ++i)
    {
        //printf("%d \n", data[i]);
        shm[i] = data[i];
        //rintf("%f\n", data[i]);
    }

}


float* SHM_SERVER::getMem(){
    return shm;
}

