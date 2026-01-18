#ifndef INC_LASER_H_
#define INC_LASER_H_

#include "main.h"
#include "posicion_pool.h"
#include "botones.h"


typedef enum { FIRE_MANUAL = 0, FIRE_AUTO } FireMode;
void set_servo_laser_horizontal(TIM_HandleTypeDef *htim, uint16_t us);
void set_servo_laser_vertical(TIM_HandleTypeDef *htim, uint16_t us);

void laser_set_estado(FireMode r);
FireMode laser_get_estado(void);

void laser_apuntar(TIM_HandleTypeDef *htim);

void laser_rotacion_mode(TIM_HandleTypeDef *htim, uint8_t* next_obj, uint8_t* fire_btn);

void laser_dispara_task(void);
void laser_dispara_start(void);


void laser_reset(TIM_HandleTypeDef *htim);

#endif /* INC_LASER_H_ */
