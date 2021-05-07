

/*
#define STANDARD_GRAVITY 9.80665f
#define DEG2RAD(deg) (deg / 180 * M_PI)

#define RES_2G 2.0f
#define RES_250DPS 250.0f
#define MAX_INT16 32768.0f
#define ACC_RAW2G (RES_2G / MAX_INT16) //2G scale for 32768 raw value
#define GYRO_RAW2DPS (RES_250DPS / MAX_INT16) //250DPS (degrees per second) scale for 32768
*/
//#ifndef GYRO_ANGLES_H_
//#define GYRO_ANGLES_H_


#define FILTERSIZE 32

//#define ADAPTATION_ROBOT PI


//void imu_compute_offset(messagebus_topic_t * imu_topic, uint16_t nb_samples);
//void imu_compute_units(void);
int get_angle_acc(float abs, float ord);
void start_imu(void);

