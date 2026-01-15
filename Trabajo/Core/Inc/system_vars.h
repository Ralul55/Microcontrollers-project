#ifndef INC_SYSTEM_VARS_H_
#define INC_SYSTEM_VARS_H_

#include <stdint.h>

// Tipos (deben estar en un .h para que los vean main.c y menus.c)
typedef enum { FIRE_MANUAL = 0, FIRE_AUTO } FireMode;
typedef enum { ROT_90 = 0, ROT_180, ROT_MANUAL } RotMode;

// Variables globales DEL SISTEMA (definidas en main.c)
extern uint16_t num_objetivos;
extern uint16_t num_abatidos;

extern FireMode modo_disparo;   // MENU 2 modifica esto
extern RotMode  modo_rotacion;  // MENU 3 modifica esto

extern uint16_t grados_rot;     // se muestra en MENU 3 (lo actualiza tu lógica)
extern uint16_t prueba_pote_2;

#endif /* INC_SYSTEM_VARS_H_ */
