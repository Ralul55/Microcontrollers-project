#include "botones.h"
#include <string.h>

#define BTN_DEBOUNCE_MS 30u
#define BTN_LONGPRESS_MS 2000u


// devuelve 1 si esta pulsado, 0 si esta suelto (independiente de si es activo LOW/HIGH)
static uint8_t leer_pulsado(const Boton *b)
{
    uint8_t nivel = (HAL_GPIO_ReadPin(b->port, b->pin) != GPIO_PIN_RESET) ? 1u : 0u;   //leemos PIN y convertimos a 1 o 0
    return (nivel == b->pressed_level) ? 1u : 0u;									   //convertimos -> pulsado siempre = 1, suelto siempre = 0
}

//inicializa los botones, asigna atributos
void Boton_Init(Boton *b, GPIO_TypeDef *port, uint16_t pin, uint8_t pressed_level)
{
    memset(b, 0, sizeof(*b)); //limpia estructura

    b->port = port;
    b->pin  = pin;
    b->pressed_level = pressed_level;

    // estado inicial
    b->boton_presionado = leer_pulsado(b);              //leemos estado actual del boton
    b->boton_deb        = b->boton_presionado;			//asignaciones iniciales para evitar incongruencias entre variables al inicializar

    b->presionado = b->boton_deb;
    b->flag_pulso_largo = 0u;
}

uint8_t Boton_IsPressed(const Boton *b)
{
    return b->presionado;
}

BtnEvent Boton_Update(Boton *b, uint32_t now_ms)
{
    BtnEvent ev = BTN_EVENT_NONE;
    uint8_t raw = leer_pulsado(b);

    //comprobamos si se ha pulsado el boton y actualizamos
    if (raw != b->boton_presionado) {
        b->boton_presionado = raw;
        b->t_last_change_ms = now_ms;
    }

    // debouncer: aceptamos cambio si la señal es establa durante 30 ms (BTN_DEBOUNCE_MS)
    if ((now_ms - b->t_last_change_ms) >= BTN_DEBOUNCE_MS && b->boton_deb != raw) {
        b->boton_deb = raw;

        if (b->boton_deb) {
            //comprobamos si sigue pulsado
            b->presionado = 1u;
            b->t_press_start_ms = now_ms;
            b->flag_pulso_largo = 0u;
        } else {
            //si estaba pulsado y soltamos -> pulso corto
            if (b->presionado && !b->flag_pulso_largo) {
                ev = BTN_EVENT_SHORT;
            }
            b->presionado = 0u;
        }
    }

    //comprobamos si es un pulso largo, y si no habia pulso largo ya establecido -> pulso largo
    if (b->presionado && !b->flag_pulso_largo &&
        (now_ms - b->t_press_start_ms) >= BTN_LONGPRESS_MS) {

        b->flag_pulso_largo = 1u;
        ev = BTN_EVENT_LONG;
    }

    return ev;
}

