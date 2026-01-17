#include "motor_laser_movimiento.h"
#include "posicion_pool.h"
#include <math.h>


static FireMode estado_actual;
#define distancia_Laser_Sensor 82 // mm
#define PI 3.141592

void set_servo_laser_horizontal(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion del motor horizontal del laser
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, us);
}

void set_servo_laser_vertical(TIM_HandleTypeDef *htim, uint16_t us){
	// Establece la posicion del motor horizontal del laser
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, us);
}

void laser_set_estado(FireMode r){
	estado_actual=r;
}

FireMode laser_get_estado(void){
	return estado_actual;
}


void laser_apuntar(TIM_HandleTypeDef *htim){
	Posicion* Objetivo;
	Objetivo = get_Objetivo();

	if (Objetivo != NULL) {
		uint16_t angulo_Laser_Horizontal = transforma_a_entero(Objetivo->angulo);
	    set_servo_laser_horizontal(htim, angulo_Laser_Horizontal);

	    uint16_t angulo_Laser_Vertical_radianes = atan2(Objetivo->distancia ,distancia_Laser_Sensor);
	    uint16_t angulo_Laser_Vertical = transforma_a_entero(angulo_Laser_Vertical_radianes * (360u/(2*PI)));
	    set_servo_laser_vertical(htim, angulo_Laser_Vertical);
	}

}


void laser_rotacion_mode(TIM_HandleTypeDef *htim ,uint8_t* flag_boton_siguiente_objetivo, uint8_t* flag_disparo){
	switch (estado_actual) {
		case FIRE_MANUAL:
			if (*flag_boton_siguiente_objetivo==1){
				laser_apuntar(htim);
				*flag_boton_siguiente_objetivo=0;
			}
			break;
		case FIRE_AUTO:
			if (*flag_disparo){
				HAL_Delay(500);
				laser_apuntar(htim);
				*flag_disparo=0;
			}
			break;
	}
}



