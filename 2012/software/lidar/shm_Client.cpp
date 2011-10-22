#include "shm_Client.hpp"
#include "shm_id.h"

SHM_CLIENT::SHM_CLIENT(int key, int size):key(key), shmSize(size){}

bool SHM_CLIENT::connect(){

    if((shmID = shmget(key, shmSize, 0666)) == NULL){
        perror("Error getting shared memory.");
        return false;
    }

    if((shm = (float*)shmat(shmID, NULL, 0)) == NULL){
        perror("Error attaching shared memory.");
        return false;
    }

    
    return true;
}


float* SHM_CLIENT::shmRead(){

    return shm;
}


