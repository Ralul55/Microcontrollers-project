<<<<<<< Updated upstream:Trabajo/Core/Src/motor_radar_movimiento.c
#include "motor_radar_movimiento.h"
=======
#include "Radar.h"
#include "posicion_pool.h"
#include "menus.h"
>>>>>>> Stashed changes:Trabajo/Core/Src/Radar.c

static uint16_t angulo_Radar_Horizontal = GIRO_MIN;
static uint8_t flag_Sentido_Horario = 1;

void set_servo_radar(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion del motor DEL CANAL 1
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, us);
}

void movimiento_radar(VL53L0X_RangingMeasurementData_t *Ranging, TIM_HandleTypeDef *htim, uint16_t step)
{
	// Evolucion de la posicion del motor por pasos
	// Si se quiere hacer un control más fino de la posicion se puede reducir el paso pero ira mas lento
    if (flag_Sentido_Horario) angulo_Radar_Horizontal += step;
    else                      angulo_Radar_Horizontal -= step;

    // un ligero control de errores para que no se pase del giro maximo/minimo permitido, y si llega al giro maximo/minimo cambia el sentido de rotacion
    if (angulo_Radar_Horizontal >= GIRO_MAX) { angulo_Radar_Horizontal = GIRO_MAX; flag_Sentido_Horario = 0; }
    if (angulo_Radar_Horizontal <= GIRO_MIN) { angulo_Radar_Horizontal = GIRO_MIN; flag_Sentido_Horario = 1; }

    // Establece la posicion del motor
    set_servo_radar(htim, angulo_Radar_Horizontal);
    // Pequeño delay para asegurarnos de que el motor llega a la posicion antes de proceder al siguiente paso en el main
    HAL_Delay(5);
    // 2) MEDIR AHORA
    if (LidarMedir(Ranging) == VL53L0X_ERROR_NONE) {
      detectar_Objetivo(Ranging, angulo_Radar_Horizontal);
    }

}

// Getter para el angulo del radar
uint16_t radar_get_angulo(void) { return angulo_Radar_Horizontal; }
<<<<<<< Updated upstream:Trabajo/Core/Src/motor_radar_movimiento.c
=======

void radar_set_estado(RotMode r){
	estado_actual=r;
}


RotMode radar_get_estado(void){
	return estado_actual;
}

void radar_rotacion_mode(uint16_t grados_rot){
	switch (estado_actual) {
		case ROT_360:
			GIRO_MAX=2500;
			break;
		case ROT_180:
			GIRO_MAX=1500;
			break;
		case ROT_MANUAL:
			GIRO_MAX=grados_rot;
			break;
	}
}

void radar_reset(TIM_HandleTypeDef *htim){
	set_servo_radar(htim, 500);
}

void guardar_con_(uint16_t dist){

}
>>>>>>> Stashed changes:Trabajo/Core/Src/Radar.c
