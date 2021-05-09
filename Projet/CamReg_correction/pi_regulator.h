#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

int direction_haut(int);

//start the PI regulator thread
void pi_regulator_start(void);
int16_t pi_regulator(int);

#endif /* PI_REGULATOR_H */

