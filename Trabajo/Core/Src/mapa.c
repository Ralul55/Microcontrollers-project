#include "mapa.h"
#include <stdlib.h>


static bool mapa_limites(int x, int y)
{
    return (x >= 0 && y >= 0 && x < (int)TAM_PANT_W && y < (int)TAM_PANT_H);
}


void mapa_init(void){
	ILI9341_Init();
	mapa_dibuja();
    //codigo de prueba
    mapa_dibuja_cuz(TAM_PANT_W / 2, TAM_PANT_H / 2, true);

    mapa_dibuja_cuz(TAM_PANT_W / 2 - 4, TAM_PANT_H / 2 - 4, false);
    mapa_dibuja_cuz(TAM_PANT_H / 2 - 7, TAM_PANT_H / 2 - 7, false);
    mapa_borra_cuz(TAM_PANT_H / 2 - 7, TAM_PANT_H / 2 - 7);


}

void mapa_rectangulo(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){

	//respeto de margenes
	if (w == 0 || h == 0) return;
	if (x >= TAM_PANT_W || y >= TAM_PANT_H) return;
	if (x + w > TAM_PANT_W) w = TAM_PANT_W - x;
	if (y + h > TAM_PANT_H) h = TAM_PANT_H - y;


	for (uint16_t yy = 0; yy < h; yy++) {
	        for (uint16_t xx = 0; xx < w; xx++) {
	            ILI9341_WritePixel(x + xx, y + yy, color);
	        }
	    }
}

void mapa_dibuja_linea(int x0, int y0, int x1, int y1, uint16_t color){

	//SE UTILIZA ALGORITMO DE BRESENHAM
	int dx=	abs(x1 - x0);;
	int dy=	abs(y1 - y0);;
	int sx=1;
	int sy=1;

	if (x0 > x1) sx = -1;
	if (y0 > y1) sy = -1;

	int err = dx - dy;

    while (1) {
        if (mapa_limites(x0, y0)) {
            ILI9341_WritePixel((uint16_t)x0, (uint16_t)y0, color);
        }
        if (x0 == x1 && y0 == y1) break; //condicion de salida

        int e2 = 2 * err;
        if (e2 > -dy) {
        	err -= dy;
        	x0 += sx;
        }
        if (e2 < dx) {
        	err += dx;
        	y0 += sy;
        }
    }
}



void mapa_dibuja_cuz_g(Posicion *pos, bool marcado){
	uint16_t coordenadas[2];
	mapa_pasar_coordenadas(pos, coordenadas);
	uint16_t x=coordenadas[0];
	uint16_t y=coordenadas[1];
	mapa_dibuja_cuz(x, y, marcado);
}


void mapa_dibuja_cuz(uint16_t x, uint16_t y, uint8_t marcado){
	uint16_t color;
	if (marcado==0) color=NEGRO;
	else if (marcado==1) color=ROJO;
	else color=VERDE;

	mapa_dibuja_linea(x-4, y-4, x+4, y+4, color);
	mapa_dibuja_linea(x-3, y-4, x+5, y+4, color);
	mapa_dibuja_linea(x-5, y-4, x+3, y+4, color);

	mapa_dibuja_linea(x-4, y+4, x+4, y-4, color);
	mapa_dibuja_linea(x-3, y+4, x+5, y-4, color);
	mapa_dibuja_linea(x-5, y+4, x+3, y-4, color);

}

void mapa_borra_cuz(uint16_t x, uint16_t y){
	//pinta una cruz del color del fondo encima
	uint8_t i = 3;
	mapa_dibuja_cuz(x, y, i);
}


//recibe posicion y devuelve puntero a un array de coordenas
void mapa_pasar_coordenadas(Posicion *pos, uint16_t coordenadas[2]){

	//codigo de conversion

}

void mapa_dibuja(void){
	mapa_rectangulo(40, 0, TAM_PANT_W-80, TAM_PANT_H, VERDE); //cuadrado
	mapa_rectangulo(TAM_PANT_W-80, 0, TAM_PANT_W-80, TAM_PANT_H, VERDE); //cuadrado


}
