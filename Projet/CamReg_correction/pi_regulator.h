#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

float direction_haut(float);
float get_angle(void);
//start the PI regulator thread
void pi_regulator_start(void);

#endif /* PI_REGULATOR_H */
