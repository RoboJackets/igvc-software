#include "shm_Server.hpp"

int main(){
printf("dkjf");
    SHM_SERVER server(0164);
    server.setup(10);
    int data[10];
    while(1){
    for (int x=0; x<10; x++)
	data[x] = rand()%255;

for(int x=0; x<20; x++)
 printf("%d, ", data[x]);
printf("\n");
    server.shmWrite(data);
	sleep(1);
}
}
