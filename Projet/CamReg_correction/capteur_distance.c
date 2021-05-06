/*
 * capteur_distance.c
 *
 *  Created on: 20 avr. 2021
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
#include <leds.h>
#include <sensors/proximity.h>

uint8_t NB_CAPTEUR= 8;
uint8_t VALEUR_REF = 0;
uint8_t num_proche = 0;
unsigned int valeur_capt_proche = 0;

static THD_WORKING_AREA(waCapteurDistance, 256);
static THD_FUNCTION(CapteurDistance, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    calibrate_ir();

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;




    while(1){
        time = chVTGetSystemTime();
        messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
//        chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.ambient[0]);
//        chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.reflected[0]);
//        chprintf((BaseSequentialStream *)&SDU1, "%4d", prox_values.delta[0]);
//        chprintf((BaseSequentialStream *)&SDU1, "\r\n");

        num_proche = VALEUR_REF;
        int marge_detection = 500; //échelle du capteur de 0 à 4000, 0 est quand le mur touche le capteur.
        valeur_capt_proche = VALEUR_REF;


//        for(uint8_t i=0; i< sizeof(prox_values.ambient)/sizeof(prox_values.ambient[0]); i++){
        for(uint8_t i=0; i< NB_CAPTEUR; i++){
        	 if((prox_values.reflected[i] + marge_detection < prox_values.ambient[i]) && (prox_values.delta[i]>=prox_values.delta[num_proche])){
        		 num_proche = i;
        		 valeur_capt_proche = prox_values.reflected[i];

//        		 	chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.ambient[i]);
//        		 	chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.reflected[i]);
//        		 	chprintf((BaseSequentialStream *)&SDU1, "%4d", prox_values.delta[i]);
//        		 	chprintf((BaseSequentialStream *)&SDU1, "\r\n");

        	 	 }//else{
//        		 valeur_capt_proche = 0;
//        		 //num_proche = 8;// attention le 8 est pour dire aucun, à changer plus tard
//        		 clear_leds();
//        	 }

        }
//        if (num_proche==0 && valeur_capt_proche != 0){
//        	set_led(LED1,TRUE); // value (0=off 1=on higher=inverse)
//       	}else if(num_proche==1){
//            set_led(LED3,TRUE);
//            set_led(LED1,TRUE);// value (0=off 1=on higher=inverse)
//        }else if(num_proche==2){
//            set_led(LED3,TRUE);// value (0=off 1=on higher=inverse)
//        }else if(num_proche==3){
//            set_led(LED5,TRUE);
//            set_led(LED3,TRUE);// value (0=off 1=on higher=inverse)
//        }else if(num_proche==4){
//            set_led(LED5,TRUE);
//            set_led(LED7,TRUE);
//        }else if(num_proche==5){
//            set_led(LED7,TRUE);
//        }else if(num_proche==6){
//            set_led(LED1,TRUE);
//            set_led(LED7,TRUE);
//        }else if(num_proche==7){
//            set_led(LED1,TRUE);
//        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }

}
void capteur_distance_start(void){
	chThdCreateStatic(waCapteurDistance, sizeof(waCapteurDistance), NORMALPRIO, CapteurDistance, NULL);
}

uint8_t get_capteur_proche(void){
	return num_proche;
}

unsigned int get_val_capteur_proche(void){
	return valeur_capt_proche;
}




