/*
 * conditions_fin.c
 *
 *  Created on: 8 mai 2021
 *      Author: Ludovic
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <stdbool.h>



#include <main.h>
#include <conditions_fin.h>
#include <gyro_angles.h>
#include <capteur_distance.h>
#include <leds.h>


static bool fin = FALSE;


enum CAPTEURS_IR {ir1, ir2,ir3,ir4,ir5,ir6,ir7,ir8};

static THD_WORKING_AREA(waConditionsFin, 256);
static THD_FUNCTION(ConditionsFin, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        int8_t angle = get_angle_dir();
        enum CAPTEURS_IR capteur_proche = get_capteur_proche(LE_PLUS_PROCHE);
        enum CAPTEURS_IR capteur_proche_2 = get_capteur_proche(LE_2ND_PLUS_PROCHE);

        if(fin == TRUE){
        	if(get_val_capteur_proche(LE_PLUS_PROCHE) !=0){
				if(((capteur_proche == ir1 ) || (capteur_proche == ir8)) && fabs(angle) >= MARGE_SORTIE_FIN) {
					fin = FALSE; // fin si le robot arrive droit sur un mur horizontale
				}else if((capteur_proche == ir3 ||capteur_proche_2 == ir3) && fabs(angle- ANGLE_DROIT) >= MARGE_SORTIE_FIN){
					fin = FALSE; // fin si le robot arrive en longeant un mur à sa droite
				}else if((capteur_proche == ir6 || capteur_proche_2 == ir6) && fabs(angle + ANGLE_DROIT) >= MARGE_SORTIE_FIN){
					fin = FALSE; // fin si le robot arrive en longeant un mur à sa gauche
				}else if (((capteur_proche == ir2 && capteur_proche_2 == ir7) && fabs(angle) >= MARGE_SORTIE_FIN)||
						((capteur_proche == ir7 && capteur_proche_2 == ir2) && fabs(angle) >= MARGE_SORTIE_FIN)){
					fin = FALSE; // fin si le robot arrive dans un angle
				}
        	}else fin = FALSE;

        }else{
			if(((capteur_proche == ir1 && get_val_capteur_proche(LE_PLUS_PROCHE) != 0) ||
					(capteur_proche == ir8)) && fabs(angle) <= MARGE_ANGLE_FIN) {
				fin = TRUE; // fin si le robot arrive droit sur un mur horizontale
			}
			if(capteur_proche == ir3 && fabs(angle - ANGLE_DROIT) <= MARGE_ANGLE_FIN){
				fin = TRUE; // fin si le robot arrive en longeant un mur à sa droite
			}
			if((capteur_proche == ir6 || capteur_proche_2 == ir6) && fabs(angle + ANGLE_DROIT) <= MARGE_ANGLE_FIN){
				fin = TRUE; // fin si le robot arrive en longeant un mur à sa droite
			}
			if (((capteur_proche == ir2 && capteur_proche_2 == ir7) && fabs(angle) <= MARGE_ANGLE_FIN)||
					((capteur_proche == ir7 && capteur_proche_2 == ir2) && fabs(angle) <= MARGE_ANGLE_FIN)){
				fin = TRUE; // fin si le robot arrive dans un angle
			}
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

