all: plant controller backup-controller reference monitor diag

plant: plant.o rt-lib.o
	gcc -o plant plant.o rt-lib.o -lrt -pthread

controller: controller.o rt-lib.o
	gcc -o controller controller.o rt-lib.o -lrt -pthread

backup-controller: backup-controller.c rt-lib.o
	gcc -o backup-controller backup-controller.c rt-lib.o -lrt -pthread

reference: reference.c parameters.h
	gcc -o reference reference.c -lrt

diag: diag.c parameters.h
	gcc -o diag diag.c -lrt

monitor: monitor.o rt-lib.o
	gcc -o monitor monitor.o rt-lib.o -lrt -pthread

plant.o: plant.c rt-lib.h parameters.h
	gcc -c plant.c

controller.o: controller.c rt-lib.h parameters.h
	gcc -c controller.c

monitor.o: monitor.c rt-lib.h parameters.h
	gcc -c monitor.c

rt-lib.o: rt-lib.c rt-lib.h
	gcc -c rt-lib.c

clean:
	rm *.o plant controller backup-controller reference monitor diag

