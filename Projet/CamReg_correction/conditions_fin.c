/*
 * conditions_fin.c
 *
 *  Created on: 8 mai 2021
 *      Author: Ludovic
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdbool.h>



#include <main.h>
#include <conditions_fin.h>
#include <gyro_angles.h>
#include <capteur_distance.h>
#include <leds.h>

uint8_t MARGE_ANGLE_FIN = 7;// pas grave si c'est large, grosse impréssision si on tient le plateaux à la main
uint8_t ANGLE_DROIT = 90;
bool fin = FALSE;


enum capteurs_ir {ir1, ir2,ir3,ir4,ir5,ir6,ir7,ir8};

static THD_WORKING_AREA(waConditionsFin, 256);
static THD_FUNCTION(ConditionsFin, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        int angle = get_angle_dir();
        enum capteurs_ir capteur_proche = get_capteur_proche(LE_PLUS_PROCHE);
        enum capteurs_ir capteur_proche_2 = get_capteur_proche(LE_2ND_PLUS_PROCHE);

        fin= FALSE;


        if(((capteur_proche == ir1 && get_val_capteur_proche(LE_PLUS_PROCHE) !=0)|| (capteur_proche == ir8 /*&& capteur_proche_2 == ir1*/))&& fabs(angle)<= MARGE_ANGLE_FIN) {
        	fin = TRUE; // fin si le robot arrive droit sur un mur horizontale
        }
        if(capteur_proche == ir3 && fabs(angle- ANGLE_DROIT)<=MARGE_ANGLE_FIN){
        	fin = TRUE; // fin si le robot arrive en longeant un mur à sa droite
        }
        if(capteur_proche == ir6 && fabs(angle+ ANGLE_DROIT)<=MARGE_ANGLE_FIN){
        	fin = TRUE; // fin si le robot arrive en longeant un mur à sa droite
        }

        set_body_led(fin);

        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void conditions_fin_start(void){
	chThdCreateStatic(waConditionsFin, sizeof(waConditionsFin), NORMALPRIO, ConditionsFin, NULL);
}

bool get_fin(void){
	return fin;
}

