/*
 * capteur_distance.h
 *
 *  Created on: 20 avr. 2021
 *      Author: Ludovic
 */

#ifndef CAPTEUR_DISTANCE_H_
#define CAPTEUR_DISTANCE_H_


void capteur_distance_start(void);
uint8_t get_capteur_proche(uint8_t);
unsigned int get_val_capteur_proche(uint8_t);
unsigned int get_delta_capteur_proche(void);

#endif /* CAPTEUR_DISTANCE_H_ */
