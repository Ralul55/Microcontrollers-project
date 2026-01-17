#include "motor_laser_movimiento.h"
#include "posicion_pool.h"


static FireMode estado_actual;

void set_servo_laser(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion del motor DEL CANAL 2
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, us);
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
	    	set_servo_laser(htim, angulo_Laser_Horizontal);
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



