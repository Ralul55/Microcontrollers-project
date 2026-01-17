#ifndef INC_MOTOR_RADAR_MOVIMIENTO_H_
#define INC_MOTOR_RADAR_MOVIMIENTO_H_

#include "main.h"   // para TIM_HandleTypeDef

typedef enum { ROT_360 = 0, ROT_180, ROT_MANUAL } RotMode;

void movimiento_radar(TIM_HandleTypeDef *htim, uint16_t step);
void set_servo_radar(TIM_HandleTypeDef *htim, uint16_t us);
uint16_t radar_get_angulo(void);

RotMode radar_get_estado(void);
void radar_set_estado(RotMode r);
void radar_rotacion_mode(uint16_t grados_rot);

#endif
