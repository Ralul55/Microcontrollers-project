#include "Laser.h"
#include "posicion_pool.h"
#include <math.h>


static FireMode estado_actual;
static uint8_t flag_disparo = 0;
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


void laser_rotacion_mode(TIM_HandleTypeDef *htim ,uint8_t* flag_boton_siguiente_objetivo, uint8_t* flag_boton_disparo){
	switch (estado_actual) {
		case FIRE_MANUAL:
			if(*flag_boton_disparo){
				laser_dispara();
				*flag_boton_disparo = 0;
			}
			if (*flag_boton_siguiente_objetivo==1){
				laser_apuntar(htim);
				*flag_boton_siguiente_objetivo=0;
			}
			break;
		case FIRE_AUTO:
			uint32_t actual;
			flag_disparo = 1;						// Para el primer disparo
			while(estado_actual == FIRE_AUTO){
				if (flag_disparo){
					flag_disparo = 0;
					actual = HAL_GetTick();
					if(HAL_GetTick() - actual >= 250){
						laser_apuntar(htim);

						if(HAL_GetTick() - actual >= 1000){ // Espera para que apunte
							laser_dispara();
						}
					}
				}

			}

			break;
	}
}

void laser_dispara(void){
	uint32_t inicio = HAL_GetTick();
	HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
	if(HAL_GetTick() - inicio >= 375){
		HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
		flag_disparo = 1;
	}

}

void laser_reset(TIM_HandleTypeDef *htim){
	set_servo_laser_horizontal(htim, 500);
	set_servo_laser_vertical(htim, 500);
}


