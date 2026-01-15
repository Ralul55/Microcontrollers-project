#include "mapa.h"
#include <stdlib.h>
#include <math.h>


static bool mapa_limites(int x, int y)
{
    return (x >= 0 && y >= 0 && x < (int)TAM_PANT_W && y < (int)TAM_PANT_H);
}


void mapa_init(void){
	ILI9341_Init();
	mapa_rectangulo(40, 0, TAM_PANT_W-80, TAM_PANT_H, FONDO); //cuadrado

	    mapa_dibuja_cuz(TAM_PANT_W / 2, TAM_PANT_H / 2, true);//habra simbolo para radar

		//codigo de prueba

	    Posicion p1;
		Posicion p2;
		Posicion p3;
	    p1.angulo=90.0f;
	    p2.angulo=0.0f;
	    p3.angulo=270.0f;
	    p1.distancia=20.0f;
	    p2.distancia=1500.0f;
	    p3.distancia=1400.0f;

	    mapa_dibuja_cuz_g(&p1);
	    mapa_dibuja_cuz_g(&p2);
	    mapa_dibuja_cuz_g(&p3);
	    mapa_borra_cuz(&p3);

}

void mapa_rectangulo(int x, int y, int w, int h, uint16_t color){

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
            ILI9341_WritePixel((int)x0, (int)y0, color);
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



void mapa_dibuja_cuz_g(Posicion *pos){
	int coordenadas[2];
	mapa_pasar_coordenadas(pos, coordenadas);
	int x=coordenadas[0];
	int y=coordenadas[1];
	mapa_dibuja_cuz(x, y, pos->marcado);
}


void mapa_dibuja_cuz(int x, int y, uint8_t marcado){
	uint16_t color;
	switch (marcado) {
		case 0: //objetivo normal
			color=NEGRO;
			break;
		case 1: //objetivo marcado por la torreta
			color=ROJO;
			break;
		case 2: //objetivo abatido
			color=GRIS;
		case 3: //borrado
			color=FONDO;
	}

	mapa_dibuja_linea(x-4, y-4, x+4, y+4, color);
	mapa_dibuja_linea(x-3, y-4, x+5, y+4, color);
	mapa_dibuja_linea(x-5, y-4, x+3, y+4, color);

	mapa_dibuja_linea(x-4, y+4, x+4, y-4, color);
	mapa_dibuja_linea(x-3, y+4, x+5, y-4, color);
	mapa_dibuja_linea(x-5, y+4, x+3, y-4, color);

}

void mapa_borra_cuz(Posicion *pos){
	//pinta una cruz del color del fondo encima
	uint8_t i = 3; //indicador de que tiene que ser el color fondo

	int coordenadas[2];
	mapa_pasar_coordenadas(pos, coordenadas);
	int x=coordenadas[0];
	int y=coordenadas[1];

	mapa_dibuja_cuz(x, y, i);
}


//recibe posicion y devuelve puntero a un array de coordenas
void mapa_pasar_coordenadas(Posicion *pos, int coordenadas[2]){
	//el centro de la pantalla
	int x0 = 40 + (int)(TAM_PANT_W - 80) / 2;
	int y0 = (int)TAM_PANT_H / 2;

	//MAXIMA DISTANCIA DE DETECCION 1500, MAXIMOS PIXELES 240 (el radar esta en el centro 120), se hace conversion de distancia
	float d=pos->distancia;
	d=(d*(TAM_PANT_H-10)/2)/DISTANCIA_DE_DETECCION;

	coordenadas[0]=x0+(d)*cos((pos->angulo)* (M_PI / 180.0)); //x
	coordenadas[1]=y0-(d)*sin((pos->angulo)* (M_PI / 180.0)); //y

}

void mapa_dibuja(void){
	for (uint8_t i = 0u; i < MAXIMO_OBJETIVOS; i++){
			if (objetivo_hueco_usado(i)) {
				Posicion *p = objetivo(i);
			    mapa_dibuja_cuz_g(p);
			}
		}
}

