/*
 * controle_avec _leds.c
 *
 *  Created on: 27 avr. 2021
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
#include <capteur_distance.h>
#include <controle_avec_leds.h>
#include <leds.h>

static THD_WORKING_AREA(waControleDistLeds, 256);
static THD_FUNCTION(ControleDistLeds, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
          time = chVTGetSystemTime();
          uint8_t num_proche = get_capteur_proche(0);

          clear_leds();

		 if (num_proche==0 && get_val_capteur_proche(0) != 0){
					set_led(LED1,TRUE); // value (0=off 1=on higher=inverse)
				}else if(num_proche==1){
					set_led(LED3,TRUE);
					set_led(LED1,TRUE);// value (0=off 1=on higher=inverse)
				}else if(num_proche==2){
					set_led(LED3,TRUE);// value (0=off 1=on higher=inverse)
				}else if(num_proche==3){
					set_led(LED5,TRUE);
					set_led(LED3,TRUE);// value (0=off 1=on higher=inverse)
				}else if(num_proche==4){
					set_led(LED5,TRUE);
					set_led(LED7,TRUE);
				}else if(num_proche==5){
					set_led(LED7,TRUE);
				}else if(num_proche==6){
					set_led(LED1,TRUE);
					set_led(LED7,TRUE);
				}else if(num_proche==7){
					set_led(LED1,TRUE);
				}
		chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void controle_distance_leds_start(void){
	chThdCreateStatic(waControleDistLeds, sizeof(waControleDistLeds), NORMALPRIO, ControleDistLeds, NULL);
}
