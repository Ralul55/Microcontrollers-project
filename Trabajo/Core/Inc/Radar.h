#ifndef INC_RADAR_H_
#define INC_RADAR_H_

#include "main.h"   // para TIM_HandleTypeDef
#include "stm32f4xx_hal.h"
#include "vl53l0x_api.h"
#include "sensor_distancia.h"

typedef enum { ROT_360 = 0, ROT_180, ROT_MANUAL } RotMode;

void movimiento_radar(VL53L0X_RangingMeasurementData_t *Ranging, TIM_HandleTypeDef *htim, uint16_t step);

void set_servo_radar(TIM_HandleTypeDef *htim, uint16_t us);
uint16_t radar_get_angulo(void);
void radar_reset(TIM_HandleTypeDef *htim);

RotMode radar_get_estado(void);
void radar_set_estado(RotMode r);
void radar_rotacion_mode(uint16_t grados_rot);

#endif /* INC_RADAR_H_ */
