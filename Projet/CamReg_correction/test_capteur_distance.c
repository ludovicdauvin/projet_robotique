/*
 * test_capteur_distance.c
 *
 *  Created on: 15 avr. 2021
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
#include <test_capteur_distance.h>
#include <leds.h>
#include <sensors/proximity.h>

static THD_WORKING_AREA(waTestDistance, 512);
static THD_FUNCTION(TestDistance, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    calibrate_ir();

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
      proximity_msg_t prox_values;

//    int luminosite=0;
    int marge = 500; // déterminer avec les valeurs lue sur le terminal

    while(1){
        time = chVTGetSystemTime();
        messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
        chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.ambient[0]);
        chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.reflected[0]);
        chprintf((BaseSequentialStream *)&SDU1, "%4d", prox_values.delta[0]);
        chprintf((BaseSequentialStream *)&SDU1, "\r\n");

        if( prox_values.reflected[0] + marge < prox_values.ambient[0]){
//        	 set_body_led(TRUE);
        set_rgb_led(0, 10, 0 ,0); // value (0=off 1=on higher=inverse)
        }else{
//        	 set_body_led(FALSE);
        	 set_rgb_led(LED2, 0, 0 ,0); // value (0=off 1=on higher=inverse)
        }
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }

//    while(1){
//    	 time = chVTGetSystemTime();
//    	 messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
//    for (uint8_t i = 0; i < sizeof(prox_values.ambient)/sizeof(prox_values.ambient[0]); i++) {
//  		        //for (uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
//  		        	chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.ambient[i]);
//  		        	chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.reflected[i]);
//  		        	chprintf((BaseSequentialStream *)&SDU1, "%4d", prox_values.delta[i]);
//  		        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
//  		        }
//    chThdSleepUntilWindowed(time, time + MS2ST(10));
//    }
}
void test_distance_start(void){
	chThdCreateStatic(waTestDistance, sizeof(waTestDistance), NORMALPRIO, TestDistance, NULL);
}
