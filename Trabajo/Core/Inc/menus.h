#ifndef INC_MENUS_H_
#define INC_MENUS_H_

#include <stdint.h>
#include "botones.h"  // BtnEvent
#include "Radar.h"
#include "Laser.h"

extern uint16_t distancia_maxima;
extern uint16_t distancia_actual;

void Menus_Init(void);													//inicializa el menu
void Menus_Task(TIM_HandleTypeDef *htim, BtnEvent evMenu, BtnEvent evSel, BtnEvent evRES, uint32_t now_ms);		//actualiza menus para que imprima por lcd

#endif /* INC_MENUS_H_ */
