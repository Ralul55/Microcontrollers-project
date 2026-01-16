#include "motor_laser_movimiento.h"

void set_servo_laser(TIM_HandleTypeDef *htim, uint16_t us)
{
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, us);
}


