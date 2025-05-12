//------------------- CONTROLLER.C ---------------------- 

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

struct shared_int {
	int value;
	pthread_mutex_t lock;
};

static struct shared_int shared_avg_sensor;
static struct shared_int shared_control;

struct time_info{
	unsigned long int WCET[3];
	struct timespec t_iniziale[3];
	struct timespec t_finale[3];
};

struct shared_diag{
	struct time_info info;
	int control_signal;
	pthread_mutex_t lock;
};

static struct shared_diag shared_diag_s;

int buffer[BUF_SIZE];
int head = 0;


void * acquire_filter_loop(void * par) {
	
	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);

	// Messaggio da prelevare dal driver
	char message [MAX_MSG_SIZE];

	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda sensor del plant in lettura 
	mqd_t sensor_qd;
	if ((sensor_qd = mq_open (SENSOR_QUEUE_NAME, O_RDONLY | O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("acquire filter loop: mq_open (sensor)");
		exit (1);
	}
	unsigned int sum = 0;
	int cnt = BUF_SIZE;
	while (keep_on_running)
	{
		wait_next_activation(th);

		pthread_mutex_lock(&shared_diag_s.lock);
		clock_gettime(CLOCK_THREAD_CPUTIME_ID,&shared_diag_s.info.t_iniziale[0]);
		pthread_mutex_unlock(&shared_diag_s.lock);

		// PRELIEVO DATI dalla coda del PLANT
		if (mq_receive(sensor_qd, message,MAX_MSG_SIZE,NULL) == -1){
			perror ("acquire filter loop: mq_receive (actuator)");	
			break;						//DEBUG
		}
		else{ 
			buffer[head] = atoi(message);
			sum += buffer[head];
			head = (head+1)%BUF_SIZE;
			cnt--;

			//printf("\t\t\t\tbuffer[%d]=%d, sum=%d\n",head,buffer[head],sum); //DEBUG

			// calcolo media sulle ultime BUF_SIZE letture
			if (cnt == 0) {
				cnt = BUF_SIZE;
				pthread_mutex_lock(&shared_avg_sensor.lock);
				shared_avg_sensor.value = sum/BUF_SIZE;
				//printf("\t\t\t\tavg_sensor.value=%d\n",shared_avg_sensor.value); //DEBUG
				pthread_mutex_unlock(&shared_avg_sensor.lock);
				sum = 0;
			}	
		}
		pthread_mutex_lock(&shared_diag_s.lock);
		clock_gettime(CLOCK_THREAD_CPUTIME_ID,&shared_diag_s.info.t_finale[0]);
		shared_diag_s.info.WCET[0]=difference_ns(&shared_diag_s.info.t_finale[0],&shared_diag_s.info.t_iniziale[0]);
		pthread_mutex_unlock(&shared_diag_s.lock);
	}		

	/* Clear */
    if (mq_close (sensor_qd) == -1) {
        perror ("acquire filter loop: mq_close sehsor_qd");
        exit (1);
    }

	return 0;
}


void * control_loop(void * par) {

	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);
	
	// Messaggio da prelevare dal reference
	char message [MAX_MSG_SIZE];
	
	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda per il reference, in lettura e non bloccante
	mqd_t reference_qd;
	if ((reference_qd = mq_open (REFERENCE_QUEUE_NAME, O_RDONLY | O_CREAT | O_NONBLOCK, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("control loop: mq_open (reference)");
		exit (1);
	}

	
	unsigned int reference = 110;

	unsigned int plant_state = 0;
	int error = 0;
	unsigned int control_action = 0;
	
	while (keep_on_running)
	{
		wait_next_activation(th);

		pthread_mutex_lock(&shared_diag_s.lock);
		clock_gettime(CLOCK_THREAD_CPUTIME_ID,&shared_diag_s.info.t_iniziale[1]);
		pthread_mutex_unlock(&shared_diag_s.lock);

		// legge il plant state 
		pthread_mutex_lock(&shared_avg_sensor.lock);
		plant_state = shared_avg_sensor.value;
		pthread_mutex_unlock(&shared_avg_sensor.lock);

		// riceve la reference (in modo non bloccante)
		if (mq_receive(reference_qd, message,MAX_MSG_SIZE,NULL) == -1){
			//printf ("No reference ...\n");							//DEBUG
		}
		else{
			//printf ("Reference received: %s.\n",message);			//DEBUG
			reference = atoi(message);
		}

		// calcolo della legge di controllo
		error = reference - plant_state;

		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
		else control_action = 3;

		// aggiorna la control action
		pthread_mutex_lock(&shared_control.lock);
		shared_control.value = control_action;
		pthread_mutex_unlock(&shared_control.lock);

		pthread_mutex_lock(&shared_diag_s.lock);
		clock_gettime(CLOCK_THREAD_CPUTIME_ID,&shared_diag_s.info.t_finale[1]);
		shared_diag_s.control_signal=control_action;
		shared_diag_s.info.WCET[1]=difference_ns(&shared_diag_s.info.t_finale[1],&shared_diag_s.info.t_iniziale[1]);
		pthread_mutex_unlock(&shared_diag_s.lock);

	}
	

	/* Clear */
    if (mq_close (reference_qd) == -1) {
        perror ("control loop: mq_close reference_qd");
        exit (1);
    }
	return 0;
}

void * actuator_loop(void * par) {

	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);

	// Messaggio da prelevare dal driver
	char message [MAX_MSG_SIZE];

	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda del monitor in scrittura 
	mqd_t monitor_qd;
	if ((monitor_qd = mq_open (MONITOR_QUEUE_NAME, O_WRONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("actuator loop: mq_open (monitor)");
		exit (1);
	}	

	unsigned int control_action = 0;
	unsigned int control = 0;
	while (keep_on_running)
	{
		wait_next_activation(th);

		pthread_mutex_lock(&shared_diag_s.lock);
		clock_gettime(CLOCK_THREAD_CPUTIME_ID,&shared_diag_s.info.t_iniziale[2]);
		pthread_mutex_unlock(&shared_diag_s.lock);

		// prelievo della control action
		pthread_mutex_lock(&shared_control.lock);
		control_action = shared_control.value;
		pthread_mutex_unlock(&shared_control.lock);
		
		switch (control_action) {
			case 1: control = 1; break;
			case 2:	control = -1; break;
			case 3:	control = 0; break;
			default: control = 0;
		}
		printf("Control: %d\n",control); //DEBUG
		sprintf (message, "%d", control);
		//invio del controllo al driver del plant
		if (mq_send (monitor_qd, message, strlen (message) + 1, 0) == -1) {
		    perror ("Sensor driver: Not able to send message to controller");
		    continue;
		}

		pthread_mutex_lock(&shared_diag_s.lock);
		clock_gettime(CLOCK_THREAD_CPUTIME_ID,&shared_diag_s.info.t_finale[2]);
		shared_diag_s.info.WCET[2]=difference_ns(&shared_diag_s.info.t_finale[2],&shared_diag_s.info.t_iniziale[2]);
		pthread_mutex_unlock(&shared_diag_s.lock);
	}
	/* Clear */
    if (mq_close (monitor_qd) == -1) {
        perror ("Actuator loop: mq_close monitor_qd");
        exit (1);
    }
	return 0;
}

void * diag (void * par){
	char message [MAX_MSG_SIZE];

	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;

	//ci apriamo la coda diag_queue in lettura
	mqd_t diag_queue;
	if ((diag_queue  = mq_open (DIAG_QUEUE_NAME, O_RDONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("monitor loop: mq_open (diag)");
		exit (1);
	}

	while(keep_on_running){
		if (mq_receive(diag_queue,message,MAX_MSG_SIZE,NULL) == -1){ 
			printf ("No message di diagnostica\n");
			continue;
		}
		//printf("message di diagnostica");
		pthread_mutex_lock(&shared_diag_s.lock);
		printf("Tempi di calcolo:\n");
        printf(" - WCET Task Acquire : %d ns\n", (int)shared_diag_s.info.WCET[0]);
        printf(" - WCET Task Control : %d ns\n", (int)shared_diag_s.info.WCET[1]);
        printf(" - WCET Task Actuate : %d ns\n", (int)shared_diag_s.info.WCET[2]);
        printf(" - Segnale di controllo corrente : %d\n", shared_diag_s.control_signal);
		pthread_mutex_unlock(&shared_diag_s.lock);
	}
	
    if (mq_close (diag_queue) == -1) {
        perror ("diag: mq_close diag_qd");
        exit (1);
    }
}

int main(void)
{
	printf("The controller is STARTED! [press 'q' to stop]\n");
 	
	pthread_t acquire_filter_thread;
    pthread_t control_thread;
    pthread_t actuator_thread;
	pthread_t diag_thread;

	pthread_mutexattr_t mymutexattr;
	pthread_mutexattr_init(&mymutexattr);
	pthread_mutexattr_setprotocol(&mymutexattr,PTHREAD_PRIO_INHERIT);

	pthread_mutex_init(&shared_avg_sensor.lock, &mymutexattr);
	pthread_mutex_init(&shared_control.lock, &mymutexattr);
	pthread_mutex_init(&shared_diag_s.lock,&mymutexattr);

	pthread_mutexattr_destroy(&mymutexattr);

	pthread_attr_t myattr;
	struct sched_param myparam;

	pthread_attr_init(&myattr);
	pthread_attr_setschedpolicy(&myattr, SCHED_FIFO);
	pthread_attr_setinheritsched(&myattr, PTHREAD_EXPLICIT_SCHED); 

	// ACQUIRE FILTER THREAD
	periodic_thread acquire_filter_th;
	acquire_filter_th.period = TICK_TIME;
	acquire_filter_th.priority = 50;

	myparam.sched_priority = acquire_filter_th.priority;
	pthread_attr_setschedparam(&myattr, &myparam); 
	pthread_create(&acquire_filter_thread,&myattr,acquire_filter_loop,(void*)&acquire_filter_th);

	// CONTROL THREAD
	periodic_thread control_th;
	control_th.period = TICK_TIME*BUF_SIZE;
	control_th.priority = 45;

	myparam.sched_priority = control_th.priority;
	pthread_attr_setschedparam(&myattr, &myparam); 
	pthread_create(&control_thread,&myattr,control_loop,(void*)&control_th);

	// ACTUATOR THREAD
	periodic_thread actuator_th;
	actuator_th.period = TICK_TIME*BUF_SIZE;
	actuator_th.priority = 45;

	pthread_attr_setschedparam(&myattr, &myparam); 
	pthread_create(&actuator_thread,&myattr,actuator_loop,(void*)&actuator_th);

	//daig THREAD
	myparam.sched_priority = 44;
	pthread_attr_setschedparam(&myattr, &myparam); 
	pthread_create(&diag_thread,&myattr,diag,NULL);


	pthread_attr_destroy(&myattr);
	
	
	/* Wait user exit commands*/
	while (1) {
   		if (getchar() == 'q') break;
  	}
	keep_on_running = 0;

	if (mq_unlink (REFERENCE_QUEUE_NAME) == -1) {
        perror ("Main: mq_unlink reference queue");
        exit (1);
    }

 	printf("The controller is STOPPED\n");
	return 0;
}




