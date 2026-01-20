#include "Laser.h"
#include "posicion_pool.h"
#include <math.h>


static FireMode estado_actual;
#define distancia_Laser_Sensor 80 // mm
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

static uint16_t entero_tal;
static Posicion* Objetivo;
static uint16_t angulo_Laser_Horizontal;
void laser_apuntar(TIM_HandleTypeDef *htim){
	Objetivo = get_Objetivo();

	if (Objetivo != NULL) {
		angulo_Laser_Horizontal = transforma_a_entero_radar(Objetivo->angulo);
		entero_tal = 3050 - angulo_Laser_Horizontal;
	    set_servo_laser_horizontal(htim, entero_tal); //-angulo_Laser_Horizontal

	    float angulo_Laser_Vertical_radianes = atan2(Objetivo->distancia ,distancia_Laser_Sensor);
	    uint16_t angulo_Laser_Vertical = transforma_a_entero_laser(angulo_Laser_Vertical_radianes * (360u/(2*PI)));
	    set_servo_laser_vertical(htim, angulo_Laser_Vertical);
	}

}

//------------------------------------------------------------------
static uint8_t laser_pulsando = 0;
static uint32_t t_laser = 0;

void laser_dispara_start(void){
  if (laser_pulsando) return;               // ya está en pulso
  laser_pulsando = 1;
  t_laser = HAL_GetTick();
  HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
}

void laser_dispara_task(void){
  if (!laser_pulsando) return;
  if (HAL_GetTick() - t_laser >= 1000){
    HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
    laser_pulsando = 0;
  }
}
//-------------------------------------------------------------------

void laser_rotacion_mode(TIM_HandleTypeDef *htim, uint8_t* next_obj, uint8_t* fire_btn)
{
  static uint32_t t0 = 0;
  static uint8_t fase = 0; // 0=idle, 2=espera_apunte, 3=disparar, 4=pausa

  // siempre mantener el pulso del láser si está activo
  laser_dispara_task();

  switch (estado_actual) {

    case FIRE_MANUAL:
      if (*fire_btn) {
        laser_dispara_start();
        objetivo_establecer_abatido(angulo_Laser_Horizontal);
        *fire_btn = 0;
      }
      if (*next_obj) {
        laser_apuntar(htim);
        *next_obj = 0;
      }
      fase = 0;
      break;

    case FIRE_AUTO:
      switch (fase) {

        case 0: // empezar ciclo
          laser_apuntar(htim);
          t0 = HAL_GetTick();
          fase = 2;
          break;

        case 2: // esperar a que el servo apunte
          if (HAL_GetTick() - t0 >= 1000) {
            laser_dispara_start();
            objetivo_establecer_abatido(angulo_Laser_Horizontal);
            t0 = HAL_GetTick();
            fase = 4;
          }
          break;

        case 4: // pausa entre disparos (y/o siguiente objetivo)
          if (HAL_GetTick() - t0 >= 250) {
            fase = 0; // repetir
          }
          break;
      }
      break;
  }
}

void laser_reset(TIM_HandleTypeDef *htim){
	set_servo_laser_horizontal(htim, 2500);
	set_servo_laser_vertical(htim, 950);
}
