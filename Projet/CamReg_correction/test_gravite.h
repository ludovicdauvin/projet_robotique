/*
 * test_gravité.h
 *
 *  Created on: 30 avr. 2021
 *      Author: Ludovic
 */

#ifndef TEST_GRAVITE_H_
#define TEST_GRAVITE_H_

void test_gravite_start(void);
int get_angle(void); // en degré de 0 à 180
float get_inclinaison(void); // de -9.81 à 9.81 avec 0 quand le robot est à l'horizonal. (les 9.81 sont approximatifs, c'est pour l'accélération g)


#endif /* TEST_GRAVITE_H_ */
