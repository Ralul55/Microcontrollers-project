#include "sensor_distancia.h"


static VL53L0X_Dev_t vl53l0x_c;		//static para poder usar desde cualquier parte de aqui
static VL53L0X_DEV Dev = &vl53l0x_c;

// Inicializacion del sensor (venía así por defecto en github, me funciona asi que prefiero no tocarlo)
static void LidarInit() {
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	VL53L0X_WaitDeviceBooted( Dev );
	VL53L0X_DataInit( Dev );
	VL53L0X_StaticInit( Dev );
	VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

// Funcion para obtener la distancia de que detecta el sensor
VL53L0X_Error LidarMedir(VL53L0X_RangingMeasurementData_t *out)
{
    return VL53L0X_PerformSingleRangingMeasurement(Dev, out);
    // Delay para seguridad
    HAL_Delay(5);
}

// Funcion para arrancar el sensor con los parametros que establece la funcion LidarInit()
// Fijarse en que se le pasa como argumento el I2C al que pertenezca el sensor, ya que es necesario saber a cual I2C pertenece para prepararlo para poder obtener I2cHandle y I2cDevAddr
void LidarPreparacionFuncionamiento(I2C_HandleTypeDef *hi2c){
	Dev->I2cHandle = hi2c;
	Dev->I2cDevAddr = 0x52;					// Direccion inicial del sensor

	// Resetea el xshut cada vez que enra en funcionamiento para evitar fallos
	HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_SET);
	HAL_Delay(50);

	LidarInit();							// Inicializa el sensor

	VL53L0X_SetDeviceAddress(Dev, 0x62);	//Establece la direccion en la que se guardan los datos en 0x62
	HAL_Delay(10);
	Dev->I2cDevAddr = 0x62;					// Direccion del sensor en la que mide sus datos
}



