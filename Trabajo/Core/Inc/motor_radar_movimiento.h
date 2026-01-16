#ifndef INC_MOTOR_RADAR_MOVIMIENTO_H_
#define INC_MOTOR_RADAR_MOVIMIENTO_H_

#include "main.h"   // para TIM_HandleTypeDef

#define GIRO_MIN  500
#define GIRO_MAX  2500

void movimiento_radar(TIM_HandleTypeDef *htim, uint16_t step);
void set_servo_radar(TIM_HandleTypeDef *htim, uint16_t us);
uint16_t radar_get_angulo(void);

#endif
