/*
 * test_gravite.c
 *
 *  Created on: 30 avr. 2021
 *      Author: Ludovic
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <test_gravite.h>
#include <leds.h>
#include <sensors/imu.h>

float threshold = 1;
float angle = 0;
float inclinaison = 0;

static THD_WORKING_AREA(waTestGravite, 256);
static THD_FUNCTION(TestGravite, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    calibrate_acc();

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;



    while(1){
        time = chVTGetSystemTime();

        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

        if(fabs(imu_values.acceleration[X_AXIS]) > threshold || fabs(imu_values.acceleration[Y_AXIS]) > threshold){
        	angle = atan2(imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS]);

        	angle += M_PI;

        if(angle > M_PI){
                    angle = -2 * M_PI + angle;
        }
        if(angle >= 0 && angle < M_PI/2){
            set_led(LED1,TRUE);
        }else if(angle >= M_PI/2 && angle < M_PI){
           	set_led(LED3,TRUE);
        }else if(angle >= -M_PI && angle < -M_PI/2){
           	set_led(LED5,TRUE);
        }else if(angle >= -M_PI/2 && angle < 0){
           	set_led(LED7,TRUE);
        }
        }else{
        	angle = 0;
        }

       inclinaison = imu_values.acceleration[Z_AXIS];

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }

}
void test_gravite_start(void){
	chThdCreateStatic(waTestGravite, sizeof(waTestGravite), NORMALPRIO, TestGravite, NULL);
}

float get_angle(void){
	return angle;
}

float get_inclinaison(void){
	return inclinaison;
}
