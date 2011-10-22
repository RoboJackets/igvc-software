#include "shm_Client.hpp"
#include "shm_id.h"


int main(){
    SHM_CLIENT angle_client(LI_ANGLE_ID,  1024);
    SHM_CLIENT client(LI_ANGLE_ID,  1024);
    SHM_CLIENT client(LI_ANGLE_ID,  1024);
    client.connect();

while(1){
float* data = client.shmRead();
for(int x=0; x<1024; x++)
   printf("%f\n",data[x]);
}
    return 0;
}
