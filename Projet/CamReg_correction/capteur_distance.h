/*
 * capteur_distance.h
 *
 *  Created on: 20 avr. 2021
 *      Author: Ludovic
 */

#ifndef CAPTEUR_DISTANCE_H_
#define CAPTEUR_DISTANCE_H_


//retourne le numéro du capteur le plus proche du mur ou du deuxième le plus proche
uint8_t get_capteur_proche(uint8_t);
//retourne la valeur du capteur le plus proche du mur ou du deuxième le plus proche
unsigned int get_val_capteur_proche(uint8_t);
//retourne la valeur de delta du capteur le plus proche du mur
unsigned int get_delta_capteur_proche(void);

void capteur_distance_start(void);

#endif /* CAPTEUR_DISTANCE_H_ */
