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

#define ANGLE_LIMITE  2//trouv�e exp�rimentalement
#define LIMITE_INCLINAISON  5
#define SPEED_MIN  150

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
