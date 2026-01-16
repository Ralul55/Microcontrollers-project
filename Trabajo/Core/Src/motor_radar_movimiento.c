#include "motor_radar_movimiento.h"

static uint16_t angulo_Radar_Horizontal = GIRO_MIN;
static uint8_t flag_Sentido_Horario = 1;

void set_servo_radar(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion del motor DEL CANAL 1
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, us);
}

void movimiento_radar(TIM_HandleTypeDef *htim, uint16_t step)
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
}

// Getter para el angulo del radar
uint16_t radar_get_angulo(void) { return angulo_Radar_Horizontal; }
