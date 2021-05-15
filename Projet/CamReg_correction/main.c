//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
#include <math.h>

//#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
//#include <usbcfg.h>
#include <main.h>
#include <motors.h>
//#include <chprintf.h>

#include <pi_regulator.h>


//nos includes ajouté
#include <conditions_fin.h>
#include <capteur_distance.h>
#include <controle_avec_leds.h>
#include <gyro_angles.h>
//#include <leds.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>
//#include <spi_comm.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);



int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);


//    usb_start();


    //starts sensors
    proximity_start();
    imu_start();
//    spi_comm_start();





	motors_init();

	conditions_fin_start();
	pi_regulator_start();


    //capteur distance
    capteur_distance_start();
    controle_distance_leds_start();
    gravite_start();

    while (1) {

    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
