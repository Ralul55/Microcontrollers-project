#include "motor_radar_movimiento.h"

static uint16_t angulo_Radar_Horizontal = GIRO_MIN;
static uint8_t flag_Sentido_Horario = 1;

void set_servo_radar(TIM_HandleTypeDef *htim, uint16_t us)
{
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, us);
}

void movimiento_radar(TIM_HandleTypeDef *htim, uint16_t step)
{
    if (flag_Sentido_Horario) angulo_Radar_Horizontal += step;
    else                      angulo_Radar_Horizontal -= step;

    if (angulo_Radar_Horizontal >= GIRO_MAX) { angulo_Radar_Horizontal = GIRO_MAX; flag_Sentido_Horario = 0; }
    if (angulo_Radar_Horizontal <= GIRO_MIN) { angulo_Radar_Horizontal = GIRO_MIN; flag_Sentido_Horario = 1; }

    set_servo_radar(htim, angulo_Radar_Horizontal);

    HAL_Delay(5);
}

uint16_t radar_get_angulo(void) { return angulo_Radar_Horizontal; }
