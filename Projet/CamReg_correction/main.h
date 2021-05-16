#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project

#define ERROR_THRESHOLD			2 // en degr�
#define KP						20.0f
#define KI 						0.3f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)


#define LE_PLUS_PROCHE          0
#define LE_2ND_PLUS_PROCHE 		1

#define ANGLE_LIMITE 			2//angle en degr�s, trouv�e exp�rimentalement
#define LIMITE_INCLINAISON  	5// angle en degr�s
#define SPEED_MIN  				150
#define MARGE_ANGLE_FIN  		5// pas grave si c'est large, grosse impr�cision si on tient le plateau � la main
#define DISTANCE_MUR			3000 // �quivaut � 1 cm
#define ANGLE_DROIT  			90 // en degr�s
#define MARGE_SORTIE_FIN 		20 // angle en degr�s

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
