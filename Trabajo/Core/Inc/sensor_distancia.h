#ifndef INC_SENSOR_DISTANCIA_H_
#define INC_SENSOR_DISTANCIA_H_

#include "main.h"
#include "vl53l0x_api.h"

VL53L0X_Error LidarMedir(VL53L0X_RangingMeasurementData_t *out);
void LidarPreparacionFuncionamiento(I2C_HandleTypeDef *hi2c);  // Esta funcion va en el USER CODE BEGIN 2
void LidarGuardarObjetivo();


#endif /* INC_SENSOR_DISTANCIA_H_ */
