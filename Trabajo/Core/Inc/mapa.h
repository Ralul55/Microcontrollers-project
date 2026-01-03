#ifndef INC_MAPA_H_
#define INC_MAPA_H_

#include "ili9341.h"
#include "posicion_pool.h"


#define TAM_PANT_W 320
#define TAM_PANT_H 240
#define NEGRO 0x0000
#define BLANCO 0xFFFF
#define ROJO 0xF000
#define VERDE 0x4A64

void mapa_init(void);

void mapa_rectangulo(int x, int y, int w, int h, uint16_t color);
void mapa_dibuja_linea(int x0, int y0, int x1, int y1, uint16_t color);

void mapa_dibuja_cuz_g(Posicion *pos, bool marcado);

void mapa_dibuja_cuz(int x, int y, uint8_t marcado);
void mapa_borra_cuz(Posicion *pos);


//recibe posicion y devuelve puntero a un array de coordenas
void mapa_pasar_coordenadas(Posicion *pos, int coordenadas[2]);


#endif /* INC_MAPA_H_ */
