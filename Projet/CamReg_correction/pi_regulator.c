#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdbool.h>


#include <main.h>
#include <conditions_fin.h>
#include <motors.h>
#include <pi_regulator.h>
#include <gyro_angles.h>
#include <capteur_distance.h>
#include <leds.h>
//#include <process_image.h>

//simple PI regulator implementation
int16_t pi_regulator(int error){

	int16_t speed=0;

	static float sum_error = 0;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

int ANGLE_LIMITE = 2;//trouvée expérimentalement
int FACTOR_STRAIGHT = 400;
int FACTOR_ROTATION = 200;
int ANGLE_FIN_ROTATION = 5;
int ANGLE_FIN_DROIT = 20;
int LIMITE_INCLINAISON = 5;
int SPEED_MIN = 150;


enum capteurs_ir {ir1, ir2,ir3,ir4,ir5,ir6,ir7,ir8};

//int direction_haut(int angle){
//	int rotation;
//	 if(fabs(angle) <= ANGLE_LIMITE){
//	   	rotation = 0;
//	 }else{
//	rotation = MOTOR_SPEED_LIMIT*angle/0.73; // pour le réglage de la vitesse le 0.73 correspond au rapport de rayon entre lesroues et l'écart entre les roues
//	 }
//	return rotation;
//}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int speed_straight=0;
    bool tout_droit = FALSE;
    static bool rotation_mur = FALSE;
    uint16_t valeur_proche_memoire = 0;


    while(1){
        time = chVTGetSystemTime();
        
        int angle = get_angle_dir();

        uint16_t valeur_proche = get_val_capteur_proche(LE_PLUS_PROCHE);
        enum capteurs_ir capteur_proche = get_capteur_proche(LE_PLUS_PROCHE);

        if((get_fin() == FALSE) && ((fabs(get_angle_inc()) > LIMITE_INCLINAISON)||(fabs(get_angle_inc_x()) > LIMITE_INCLINAISON))){

			if((((capteur_proche == ir1 && valeur_proche!=0)||capteur_proche == ir2||capteur_proche == ir3) && angle > 0) || ((capteur_proche == ir6||capteur_proche == ir7||capteur_proche == ir8) && angle <= 0)){

				if ((capteur_proche == ir1||capteur_proche == ir2||capteur_proche == ir8||capteur_proche == ir7) && valeur_proche > 3100){ // marge mise pour s'assurer que le capteur 3 ou 6 détecte pendant la rotation
					capteur_proche = ir1;
					valeur_proche = 0;
				}

				if(valeur_proche != 0 || rotation_mur){
					switch (capteur_proche){
						case ir1: case ir2:
							rotation_mur = TRUE;
						break;

						case ir7: case ir8:
							rotation_mur = TRUE;
						break;

						case ir3:

							if(get_val_capteur_proche(LE_PLUS_PROCHE)>valeur_proche_memoire && valeur_proche_memoire != 0){
								rotation_mur = FALSE;
								valeur_proche_memoire = 0; // valeur plus grand que le max
							}else{
								valeur_proche_memoire = get_val_capteur_proche(LE_PLUS_PROCHE);
							}
							if (rotation_mur == FALSE){
								if(get_val_capteur_proche(LE_PLUS_PROCHE) < 2800 ){ // mettre une limite à finir projet de suivre en zig zag entre deux valeurs de seuil.
									right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
									left_motor_set_speed(MOTOR_SPEED_LIMIT/4);
								}else if(get_val_capteur_proche(LE_PLUS_PROCHE) > 2900){
									right_motor_set_speed(MOTOR_SPEED_LIMIT/4);
									left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
								}else{
									right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
									left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
								}
							}

						break;

						case ir6:
							if(get_val_capteur_proche(LE_PLUS_PROCHE)>valeur_proche_memoire && valeur_proche_memoire != 0){// vu qu on ne touche jamais le mur, la variable 0 n'est pas possible en fonctionnement normal
								rotation_mur = FALSE;
								valeur_proche_memoire = 0;
							}else{
								valeur_proche_memoire = get_val_capteur_proche(LE_PLUS_PROCHE);
							}
							if (rotation_mur == FALSE){
								if(get_val_capteur_proche(LE_PLUS_PROCHE) < 2800 ){ // mettre une limite à finir projet de suivre en zig zag entre deux valeurs de seuil.
									right_motor_set_speed(MOTOR_SPEED_LIMIT/4);
									left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
								}else if(get_val_capteur_proche(LE_PLUS_PROCHE) > 3000){
									right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
									left_motor_set_speed(MOTOR_SPEED_LIMIT/4);
								}else{
									right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
									left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
								}
							}
						break;

						default:
							right_motor_set_speed(0);
							left_motor_set_speed(0);

					}
					if(rotation_mur && (capteur_proche == ir1 || capteur_proche == ir2 ||capteur_proche == ir3) && valeur_proche !=0){
						right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
						left_motor_set_speed(-MOTOR_SPEED_LIMIT/2);
					}
					if(rotation_mur && (capteur_proche == ir8 || capteur_proche == ir7 ||capteur_proche == ir6)){
						right_motor_set_speed(-MOTOR_SPEED_LIMIT/2);
						left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
					}

				}

			}else{
//				 if(fabs(angle) >= ANGLE_FIN_ROTATION && tout_droit == FALSE){
//						if(angle>0){
//							right_motor_set_speed(- fabs(MOTOR_SPEED_LIMIT *angle/180)-SPEED_MIN);
//							left_motor_set_speed(+ fabs(MOTOR_SPEED_LIMIT *angle/180)+SPEED_MIN);
//						}else if (angle<0){
//							right_motor_set_speed( fabs(MOTOR_SPEED_LIMIT *angle/180)+SPEED_MIN);
//							left_motor_set_speed(- fabs(MOTOR_SPEED_LIMIT *angle/180)-SPEED_MIN);
//						}
//
//				   }else if(get_angle_inc() > LIMITE_INCLINAISON){
//					   tout_droit = TRUE;
//
//					   right_motor_set_speed(MOTOR_SPEED_LIMIT);
//					   left_motor_set_speed( MOTOR_SPEED_LIMIT);
//
//					   if(fabs(get_angle_dir()) >= ANGLE_FIN_DROIT) tout_droit = FALSE;
//				   }else {
//						right_motor_set_speed(0);
//						left_motor_set_speed(0);
//				   }
//				if(fabs(get_angle_dir()) >= ANGLE_FIN_DROIT) {
//					right_motor_set_speed(-pi_regulator(angle));
//					left_motor_set_speed(pi_regulator(angle));
//				}else if(get_angle_inc() > LIMITE_INCLINAISON){
//				   right_motor_set_speed(MOTOR_SPEED_LIMIT);
//				   left_motor_set_speed( MOTOR_SPEED_LIMIT);
//				}
				int16_t vitesse_droit = MOTOR_SPEED_LIMIT-fabs(pi_regulator(angle));
				right_motor_set_speed((vitesse_droit-pi_regulator(angle))/2);
				left_motor_set_speed((vitesse_droit+pi_regulator(angle))/2);
			}
        }else{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
        }
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
