#ifndef INC_MOTOR_LASER_MOVIMIENTO_H_
#define INC_MOTOR_LASER_MOVIMIENTO_H_

#include "main.h"
#include "posicion_pool.h"


void set_servo_laser(TIM_HandleTypeDef *htim, uint16_t us);

typedef enum { FIRE_MANUAL = 0, FIRE_AUTO } FireMode;

FireMode laser_get_estado(void);
void laser_set_estado(FireMode r);
void laser_rotacion_mode(TIM_HandleTypeDef *htim ,uint8_t* flag_boton_siguiente_objetivo, uint8_t* flag_disparo);

#endif /* INC_MOTOR_LASER_MOVIMIENTO_H_ */
