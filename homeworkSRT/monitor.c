//------------------- MONITOR.C ---------------------- 

#include <time.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mqueue.h>
#include <fcntl.h>
#include <string.h>
#include "rt-lib.h"
#include "parameters.h"

//emulates the controller

static int keep_on_running = 1;
static int controllo=1;


void * monitor_loop(void * par) {

	// Messaggio da ricevere dal controller e inviare al plant
	char message [MAX_MSG_SIZE];
	char message2 [] = "inoltra";

	/* Code */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;

	// Apriamo una coda in sola lettura (O_RDONLY), se non esiste la creiamo (O_CREAT)
	// La coda conterrà il segnale di controllo da girare all'actuator
	mqd_t monitor_qd;
	if ((monitor_qd = mq_open (MONITOR_QUEUE_NAME, O_RDONLY | O_CREAT, QUEUE_PERMISSIONS, &attr)) == -1) {
		perror ("monitor loop: mq_open (actuator)");
		exit (1);
	}
	
	// Apriamo la coda actuator del plant in scrittura 
	mqd_t actuator_qd;
	if ((actuator_qd = mq_open (ACTUATOR_QUEUE_NAME, O_WRONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("monitor loop: mq_open (actuator)");
		exit (1);
	}	

	// Apriamo la coda del ps in scrittura 
	mqd_t ps_queue;
	if ((ps_queue  = mq_open (PS_QUEUE_NAME, O_WRONLY|O_CREAT | O_NONBLOCK, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("monitor loop: mq_open (polling server)");
		exit (1);
	}
	
	while (keep_on_running)
	{
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		timespec_add_us(&ts, TICK_TIME*BUF_SIZE+TICK_TIME*BUF_SIZE/4);
		if (mq_timedreceive(monitor_qd, message,MAX_MSG_SIZE,NULL,&ts) == -1){
			perror ("monitor loop: mq_receive (actuator)");	
			if(mq_send(ps_queue, message2, strlen(message2)+1,PS_PRIO)==-1){
				perror ("monitor loop: Not able to send message to polling server");
			}
			break;						//DEBUG
		} else {
			//invio del controllo al driver del plant
			printf("Forwarding control %s\n",message); //DEBUG
			if (mq_send (actuator_qd, message, strlen (message) + 1, 0) == -1) {
		    	perror ("monitor loop: Not able to send message to controller");
		    	continue;
			}
		}	
	}

	while (keep_on_running)
	{
		if (mq_receive(monitor_qd, message,MAX_MSG_SIZE,NULL) == -1){
			perror ("monitor loop: mq_receive (actuator)");	
			break;						//DEBUG
		} else {
			//invio del controllo al driver del plant
			printf("Forwarding control %s\n",message); //DEBUG
			if (mq_send (actuator_qd, message, strlen (message) + 1, 0) == -1) {
		    	perror ("monitor loop: Not able to send message to controller");
		    	continue;
			}
		}	
	}

/* Clear */
	if (mq_close (ps_queue) == -1) {
        perror ("monitor loop: mq_close ps_queue");
        exit (1);
    }

    if (mq_close (actuator_qd) == -1) {
        perror ("monitor loop: mq_close actuator_qd");
        exit (1);
    }
	if (mq_close (monitor_qd) == -1) {
        perror ("monitor loop: mq_close monitor_qd");
        exit (1);
    }
	return 0;
}

//ora creiamo il thread polling server

void* ps(void* parameter){
	periodic_thread *th = (periodic_thread *) parameter;
	start_periodic_timer(th,TICK_TIME);
//mesaggio che vogliamo inviare a backup
	char message [] = "attiva";

	//Messaggio da ricevere dal monitor
	char in_buffer [MAX_MSG_SIZE];

	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;

	//ci apriamo la coda ps_queue in lettura
	mqd_t ps_queue;
	if ((ps_queue  = mq_open (PS_QUEUE_NAME, O_RDONLY|O_CREAT | O_NONBLOCK, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("monitor loop: mq_open (ps)");
		exit (1);
	}

	//ci apriamo la coda baclup_queue in scrittura
	mqd_t backup_queue;
	if ((backup_queue  = mq_open (BACKUP_QUEUE_NAME, O_WRONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("monitor loop: mq_open (backup)");
		exit (1);
	}

	//ci apriamo la coda diag_queue in scrittura
	mqd_t diag_queue;
	if ((diag_queue  = mq_open (DIAG_QUEUE_NAME, O_WRONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("monitor loop: mq_open (diag)");
		exit (1);
	}
	//ci apriamo la coda diag2_queue in scrittura
	mqd_t diag2_queue;
	if ((diag2_queue  = mq_open (DIAG2_QUEUE_NAME, O_WRONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("monitor loop: mq_open (diag2)");
		exit (1);
	}

	//ora implemento il POLLING :
	while (1)
	{
		wait_next_activation(th);
		//printf("sono qui");
	    if (mq_receive(ps_queue,in_buffer,MAX_MSG_SIZE,NULL) == -1){ 
		    //printf("eccomi");
			//printf ("No message ...\n");
			continue;
	    }
	    else{
			if(strcmp(in_buffer,"inoltra")==0){
				printf ("Polling Server: message received: %s.\n",in_buffer);
				controllo = 2;		
			    if(mq_send(backup_queue,message,strlen (message)+1,0)==-1){
				    perror ("polling server: Not able to send message to backup controller");
				    continue;
			    }else{
				    //printf("mandato");
			    }
			}else{
				//printf("diag arrivato");
				if(controllo==1){
					if(mq_send(diag_queue,in_buffer,strlen (in_buffer)+1,0)==-1){
				        perror ("polling server: Not able to send message to backup controller");
					}
			    }else{
					if(mq_send(diag2_queue,in_buffer,strlen (in_buffer)+1,0)==-1){
				        perror ("polling server: Not able to send message to backup controller");
				    }
				}
	        }
	    }
	}
	if (mq_close (ps_queue) == -1) {
        perror ("monitor loop: mq_close ps_queue");
        exit (1);
    }
	if (mq_close (backup_queue) == -1) {
        perror ("monitor loop: mq_close backup_queue");
        exit (1);
    }
	if (mq_close (diag_queue) == -1) {
        perror ("monitor loop: mq_close diag_queue");
        exit (1);
    }
	if (mq_close (diag2_queue) == -1) {
        perror ("monitor loop: mq_close diag2_queue");
        exit (1);
    }
}


int main(void)
{
	printf("The monitor is STARTED! [press 'q' to stop]\n");
 	
    pthread_t monitor_thread;

	pthread_attr_t myattr;
	struct sched_param myparam;

	// MONITOR THREAD
	pthread_attr_init(&myattr);
	pthread_attr_setschedpolicy(&myattr, SCHED_FIFO);
	pthread_attr_setinheritsched(&myattr, PTHREAD_EXPLICIT_SCHED); 

	myparam.sched_priority = 51;
	pthread_attr_setschedparam(&myattr, &myparam); 
	pthread_create(&monitor_thread,&myattr,monitor_loop,NULL);
 
	//POLLING THREAD
	periodic_thread th_ps;
	pthread_t thread_ps;

	th_ps.period = TICK_TIME*BUF_SIZE;
	th_ps.priority = 53;
	myparam.sched_priority = th_ps.priority;

	pthread_attr_setschedparam(&myattr, &myparam); 	
	pthread_create(&thread_ps, &myattr, ps, (void*)&th_ps);

	pthread_attr_destroy(&myattr);

	
	/* Wait user exit commands*/
	while (1) {
   		if (getchar() == 'q') break;
  	}
	keep_on_running = 0;
	controllo=1;

	if (mq_unlink (DIAG_QUEUE_NAME) == -1) {
        perror ("Main: mq_unlink diag queue");
        exit (1);
    }

	if (mq_unlink (DIAG2_QUEUE_NAME) == -1) {
        perror ("Main: mq_unlink diag2 queue");
        exit (1);
    }

	if (mq_unlink (MONITOR_QUEUE_NAME) == -1) {
        perror ("Main: mq_unlink monitor queue");
        exit (1);
    }

	
 	printf("The monitor is STOPPED\n");
	return 0;
}




