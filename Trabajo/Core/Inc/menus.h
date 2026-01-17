#ifndef INC_MENUS_H_
#define INC_MENUS_H_

#include <stdint.h>
#include "botones.h"  // BtnEvent
#include "motor_radar_movimiento.h"
#include "motor_laser_movimiento.h"


void Menus_Init(void);													//inicializa el menu
void Menus_Task(BtnEvent evMenu, BtnEvent evSel, uint32_t now_ms);		//actualiza menus para que imprima por lcd

#endif /* INC_MENUS_H_ */
