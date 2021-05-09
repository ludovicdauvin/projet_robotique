
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors/imu.h>
#include <gyro_angles.h>
#include <main.h>

float CONVERSION_RAD_DEG = 180/M_PI;
int16_t CORRECTION_ANGLE_INC = 180;// sert à mettre 0 quand le robot est horizontal
uint16_t threshold = 1200;
int angle_dir=0;
int angle_inc=0;




static THD_WORKING_AREA(imu_reader_thd_wa, 512);
static THD_FUNCTION(imu_reader_thd, arg) {

     chRegSetThreadName(__FUNCTION__);
     (void) arg;
     systime_t time;

     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;


     calibrate_acc();

     while (1) {
    	time = chVTGetSystemTime();
    	int angle_dir_test =0;
    	int angle_inc_test =0;

		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
//		int16_t x = get_acc_filtered(X_AXIS, FILTERSIZE);
//		int16_t y = get_acc_filtered(Y_AXIS, FILTERSIZE);
//		int16_t z = get_acc_filtered(Z_AXIS, FILTERSIZE);
		int16_t x = get_acc(X_AXIS);
		int16_t y = get_acc(Y_AXIS);
		int16_t z = get_acc(Z_AXIS);

		if(fabs(y) > threshold || fabs(z+ 16000) > threshold){
		angle_inc = -atan2(y,z)*CONVERSION_RAD_DEG + CORRECTION_ANGLE_INC;
		}else{
			angle_inc = 0;
		}

		if(fabs(x) > threshold || fabs(y) > threshold){
			angle_dir = atan2(x, y)*CONVERSION_RAD_DEG;
		}else{
			angle_dir = 0;
		}

		if(angle_dir > M_PI*CONVERSION_RAD_DEG){
					angle_dir = -2 * M_PI*CONVERSION_RAD_DEG + angle_dir;
		}
		if(angle_inc > M_PI*CONVERSION_RAD_DEG){
					angle_inc = -2 * M_PI*CONVERSION_RAD_DEG + angle_inc;
		}
		angle_dir_test = angle_dir;
		angle_inc_test = angle_inc;

		chThdSleepUntilWindowed(time, time + MS2ST(50));
	}
}

void gravite_start(void){
	chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa), NORMALPRIO, imu_reader_thd, NULL);
}
int get_angle_inc(void){
	 return angle_inc;
}

int get_angle_dir(void){
	return angle_dir;
}
