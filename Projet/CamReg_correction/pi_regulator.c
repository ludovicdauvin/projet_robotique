#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdbool.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <test_gravite.h>
#include <capteur_distance.h>
#include <leds.h>
//#include <process_image.h>

//simple PI regulator implementation
//int16_t pi_regulator(float distance, float goal){
//
//	float error = 0;
//	float speed = 0;
//
//	static float sum_error = 0;
//
//	error = distance - goal;
//
//	//disables the PI regulator if the error is to small
//	//this avoids to always move as we cannot exactly be where we want and
//	//the camera is a bit noisy
//	if(fabs(error) < ERROR_THRESHOLD){
//		return 0;
//	}
//
//	sum_error += error;
//
//	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
//	if(sum_error > MAX_SUM_ERROR){
//		sum_error = MAX_SUM_ERROR;
//	}else if(sum_error < -MAX_SUM_ERROR){
//		sum_error = -MAX_SUM_ERROR;
//	}
//
//	speed = KP * error + KI * sum_error;
//
//    return (int16_t)speed;
//}

int ANGLE_LIMITE = 11;//trouvée expérimentalement
int FACTOR_STRAIGHT = 400;
int FACTOR_ROTATION = 200;
int ANGLE_FIN_ROTATION = 5;
int ANGLE_FIN_DROIT = 20;
float LIMITE_INCLINAISON = 0.5;
int SPEED_MIN = 150;

enum capteurs_ir {ir1, ir2,ir3,ir4,ir5,ir6,ir7,ir8};

int direction_haut(int angle){
	int rotation;
	 if(fabs(angle) <= ANGLE_LIMITE){
	   	rotation = 0;
	 }else{
	rotation = MOTOR_SPEED_LIMIT*angle/0.73; // pour le réglage de la vitesse le 0.73 correspond au rapport de rayon entre lesroues et l'écart entre les roues
	 }
	return rotation;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int speed_straight=0;
    int direction = 0;
    bool tout_droit = FALSE;


    while(1){
        time = chVTGetSystemTime();
        
        int angle = get_angle();
        speed_straight = get_inclinaison()*FACTOR_STRAIGHT; // mettre limite pour éviter le dépassement de capacité du int

        direction = direction_haut(angle)*FACTOR_ROTATION;



        enum capteurs_ir capteur_proche = get_capteur_proche();

        if(get_val_capteur_proche() != 0){
			switch (capteur_proche){
				case ir1: case ir2:
					right_motor_set_speed(MOTOR_SPEED_LIMIT/4);
					left_motor_set_speed(-MOTOR_SPEED_LIMIT/4);
				  // statements
				 break;

				case ir7: case ir8:
					right_motor_set_speed(-MOTOR_SPEED_LIMIT/4);
					left_motor_set_speed(MOTOR_SPEED_LIMIT/4);

				  // statements
				 break;

				case ir3:
					if(get_val_capteur_proche() < 2500){ // mettre une limite
						right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
						left_motor_set_speed(MOTOR_SPEED_LIMIT/4);
					}else{
						right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
						left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
					}
				break;

				 case ir6:
					if(get_val_capteur_proche() < 2500){ // mettre une limite
						right_motor_set_speed(MOTOR_SPEED_LIMIT/4);
						left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
					}else{
						right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
						left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
					}
				break;

				 default:
					right_motor_set_speed(0);
					left_motor_set_speed(0);

			}
        }else{
        	 if(fabs(get_angle()) >= ANGLE_FIN_ROTATION && tout_droit == 0){
					if(angle>0){
						right_motor_set_speed(- fabs(MOTOR_SPEED_LIMIT *angle/180)-SPEED_MIN);
						left_motor_set_speed(+ fabs(MOTOR_SPEED_LIMIT *angle/180)+SPEED_MIN);
					}else if (angle<0){
						right_motor_set_speed( fabs(MOTOR_SPEED_LIMIT *angle/180)+SPEED_MIN);
						left_motor_set_speed(- fabs(MOTOR_SPEED_LIMIT *angle/180)-SPEED_MIN);
					}
//					if(angle > M_PI){
//										angle = -2 * M_PI + angle;
//							}
//							if(angle >= 0 && angle < M_PI/2){
//								set_led(LED1,TRUE);
//							}else if(angle >= M_PI/2 && angle < M_PI){
//								set_led(LED3,TRUE);
//							}else if(angle >= -M_PI && angle < -M_PI/2){
//								set_led(LED5,TRUE);
//							}else if(angle >= -M_PI/2 && angle < 0){
//								set_led(LED7,TRUE);
//							}
			   }else if(get_inclinaison() > LIMITE_INCLINAISON){
				   tout_droit = TRUE;
	//				right_motor_set_speed(speed_straight);  // pas possible car le robot glisse
	//				left_motor_set_speed(speed_straight);
				   right_motor_set_speed(MOTOR_SPEED_LIMIT);
				   left_motor_set_speed( MOTOR_SPEED_LIMIT);

					if(fabs(get_angle()) >= ANGLE_FIN_DROIT) tout_droit = FALSE;
			   }else {
					right_motor_set_speed(0);
					left_motor_set_speed(0);
			   }
		}



        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
