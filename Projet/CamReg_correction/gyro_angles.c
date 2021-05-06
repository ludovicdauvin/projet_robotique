
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
//#include <test_capteur_distance.h>
//#include <sensors/proximity.h>

/*
void imu_compute_offset(messagebus_topic_t * imu_topic, uint16_t nb_samples){

 //tableau temporaire pour stocker la somme pour la moyenne
 int32_t temp_acc_offset[NB_AXIS] = {0};

 //sums nb_samples
 for(uint16_t i = 0 ; i < nb_samples ; i++)
 {
	 //nouvelles mesures pour l'IMU avec la lib messagebus
	 messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
	 for(uint8_t j = 0 ; j < NB_AXIS ; j++)
	 {
		 temp_acc_offset[j] += imu_values.acc_raw[j];
	 }
 }
 //finishes the average by dividing the sums by nb_samples
 //then stores the values to the good fields of imu_values to keep them
 for(uint8_t j = 0 ; j < NB_AXIS ; j++){
	 temp_acc_offset[j] /= nb_samples;
	 imu_values.acc_offset[j] = temp_acc_offset[j];
 }
 //specific case for the z axis because it should not be zero but -1g
 //deletes the standard gravity to have only the offset

 //void imu_compute_units(void){
// for(uint8_t i = 0 ; i < NB_AXIS ; i++){
// imu_values.acceleration[i] = ( (imu_values.acc_raw[i] - imu_values.acc_offset[i])
		 	 	 	 	 //	 * STANDARD_GRAVITY * ACC_RAW2G);
 //imu_values.gyro_rate[i] = ( (imu_values.gyro_raw[i] - imu_values.gyro_offset[i])
		 	 	 	 	 //	 * DEG2RAD(GYRO_RAW2DPS) );
 //	 }
 // }

  */

//static int16_t x = get_gyro_filtered(X_AXIS, FILTERSIZE);
//static int16_t y = get_gyro_filtered(Y_AXIS, FILTERSIZE);
float get_angle_gyro(float x,float z){
	float angle = acos(x/z);
	// angle += ADAPTATION_ROBOT;
	 return angle;
 }



//static BSEMAPHORE_DECL(detection_anglelimite_sem, TRUE);
//static bool stop = false;
//static bool angle_lim = false;
// static mode_puck_t mode = HALT;



static THD_WORKING_AREA(imu_reader_thd_wa, 512);
static THD_FUNCTION(imu_reader_thd, arg) {

     chRegSetThreadName(__FUNCTION__);
     (void) arg;
     event_listener_t imu_int;
     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
         imu_msg_t imu_values;


          /* Starts waiting for the external interrupts. */
     //    chEvtRegisterMaskWithFlags(&exti_events, &imu_int,
                                    // (eventmask_t)EXTI_EVENT_IMU_INT,
                                     //(eventflags_t)EXTI_EVENT_IMU_INT);



     //systime_t time;
     calibrate_acc();
     //float *accel = imu_values->acceleration;
     while (1) {

    	 	 messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	 	chprintf((BaseSequentialStream *)&SDU1, "IMU\r\n");
    	 	    chprintf((BaseSequentialStream *)&SDU1, "%Ax=%f Ay=%f Az=%f angle=%f \r\n\n", imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS], get_angle_gyro(imu_values.acceleration[X_AXIS], imu_values.acceleration[Z_AXIS]));
    	 	int16_t x = get_acc_filtered(X_AXIS, FILTERSIZE);
    	    int16_t z = get_acc_filtered(Z_AXIS, FILTERSIZE);
    	     get_angle_gyro(x,z);
        	chThdSleepMilliseconds(1000);
    	 		}
	}

void start_imu(void){
	chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa), NORMALPRIO, imu_reader_thd, NULL);
}




//void chThdTerminate(thread_t *tp) {

  //chSysLock();
  //tp->p_flags |= CH_FLAG_TERMINATE;
  //chSysUnlock();
//}
