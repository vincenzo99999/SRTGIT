#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <mqueue.h>
#include <fcntl.h>
#include <string.h>
#include "parameters.h"

struct timespec ts;


int main(int argc, char ** argv)
{
    ts.tv_sec=0;
    ts.tv_nsec=T_MS*NS;

    char message [] = "diag";

    struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;

   // Apriamo la coda del ps in scrittura 
	mqd_t ps_queue;
	if ((ps_queue  = mq_open (PS_QUEUE_NAME, O_WRONLY|O_CREAT | O_NONBLOCK, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("Diag: mq_open (polling server)");
		exit (1);
	}
    
while(1){

    if(nanosleep(&ts,NULL)==-1){
        printf("diag è stato interrotto");
        continue;
    }else{
        if (mq_send (ps_queue, message, strlen (message) + 1, DIAG_PRIORITY) == -1) {
            perror ("monitor loop: Not able to send message to controller");
            continue;
        }
    }
}
if (mq_close (ps_queue) == -1) {
        perror ("monitor loop: mq_close ps_queue");
        exit (1);
    }

    return 0;
}