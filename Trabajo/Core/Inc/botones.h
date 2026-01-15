#ifndef INC_BOTONES_H_
#define INC_BOTONES_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_SHORT,
    BTN_EVENT_LONG
} BtnEvent;

typedef struct {
	//variables configuracion
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t pressed_level;          //nivel eléctrico que significa "pulsado" -> en pincipio 0

    //variables debouncer
    uint8_t boton_presionado;       //ultima lectura, sin debounce
    uint8_t boton_deb;			    //señal ya debounceada
    uint32_t t_last_change_ms;		//momento en que la ultima lectura sin debounce cambió

    //variables para distinguir pulso largo y corto
    uint8_t presionado;             //boton pulsado de forma estable en momento actual
    uint32_t t_press_start_ms;		//momento de deteccion de señal estable
    uint8_t flag_pulso_largo;		//bandera para no repetir evento LONG mientras el boton esta pulsado
} Boton;

void Boton_Init(Boton *b, GPIO_TypeDef *port, uint16_t pin, uint8_t pressed_level);
BtnEvent Boton_Update(Boton *b, uint32_t now_ms);

//estado estable actual
uint8_t Boton_IsPressed(const Boton *b);

#endif /* INC_BOTONES_H_ */
