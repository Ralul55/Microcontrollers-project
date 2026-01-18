#ifndef INC_POSICION_POOL_H_
#define INC_POSICION_POOL_H_

#include <stdint.h>
#include <stdbool.h> //para poder usar bool
#include <stddef.h> //para poder usar NULL
#include "vl53l0x_api.h"

#define MAXIMO_OBJETIVOS 20u //los maximos objetivos
#define OBJETIVO_NO_ENCONTRADO 255u

typedef struct
{
	float distancia;
	float angulo; //en grados
	uint8_t marcado; //si esta marcado como objetivo o no

} Posicion;

//vamos a crear una fallsa memoria dinamica, se definen unicamente funciones para su correcto uso
//todas empiezan por objetivo para que sean mas faciles de buscar

//SE USAN BOOLS PARA TENER UNA FORMA DE VER SI FALLA, PERO PODRIAN SER VOID(!!!)
//las funciones _g son las que reciben el angulo en grados, si no recibe el valor entre 1000 (0º) y 2000 (360º)

void pool_init(void);
// Reserva hueco
bool objetivo_guarda_g(float distancia, float angulo);
//reserva hueco y hace transformacion a angulo
bool objetivo_guarda(float distancia, uint16_t angulo);
//Transformacion de grados a el valor entre 1000 (0º) y 2000 (360º)
uint16_t transforma_a_entero(float angulo);

//libera hueco segun indice
bool objetivo_libera_indice(uint8_t indice);
//libera hueco segun la posicion del angulo, en mi cabeza asi es como funcionaria
bool objetivo_libera_g(float angulo);
bool objetivo_libera(uint16_t angulo);

//mira si el hueco esta ocupado
//muy importante para recorrer el vector y saltarse el hueco vacio
bool objetivo_hueco_usado(uint8_t indice);

//devuelve la informacion de posicion
Posicion* objetivo(uint8_t indice);
Posicion* get_Objetivo(void);

//comprobacion de objetivo existente
bool objetivo_existente(uint16_t angulo);

//funciones para cosas mas "inutiles"
uint8_t objetivo_capacidad_total(void);
uint8_t objetivo_objetivos_total(void);
uint8_t objetivo_abatidos_total(void);

//para obtener indice de objetivo en posicion concreta
uint8_t objetivo_indice_angulo_g(float angulo);
uint8_t objetivo_indice_angulo(uint16_t angulo);

void objetivo_establecer_abatido(uint16_t angulo);

void pool_reset(void);

void detectar_Objetivo(VL53L0X_RangingMeasurementData_t *Ranging, uint16_t angulo_actual);

#endif /* INC_POSICION_POOL_H_ */
