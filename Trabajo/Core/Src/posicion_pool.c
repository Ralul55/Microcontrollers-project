#include "posicion_pool.h"
#include <math.h>

//gestion de la informacion estatica, solo 1 instancia
static Posicion datos[MAXIMO_OBJETIVOS];
static uint8_t huecos_ocupados[MAXIMO_OBJETIVOS]; //se encarga de anotar si el hueco esta libre (0) o ocupado (1)
static uint8_t numero_huecos = 0u;

//se necesita porque aparentemente comparar 2 floats no es trivial
static const float margen_igualdad = 0.01f;  // 0,01 grados

//1000 es 0, 2000 es 360
//es static, solo se puede usar desde aqui, si se necesita fuera pues se quita el static
static float transforma_g(uint16_t angulo){
	//comprobaciones que no se si deberian estar
	//if (angulo<1000){return 0.0f;}
	//if (angulo>2000){return 360.0f;}
	float resultado=(((float)angulo*0.36f)-360.0f);
	return resultado;
}


void pool_init(void){
	for (uint8_t i = 0u; i < MAXIMO_OBJETIVOS; i++){
		huecos_ocupados[i] = 0u;
		datos[i].distancia = 0.0f;
		datos[i].angulo = 0.0f;
	}
	numero_huecos = MAXIMO_OBJETIVOS;
}

/////////////////////////////////
//reserva hueco
bool objetivo_guarda_g(float distancia, float angulo){
	//si no hay hueco retorna falso
	if (numero_huecos == 0u){ return false; }

	for (uint8_t i = 0u; i < MAXIMO_OBJETIVOS; i++){
		if (huecos_ocupados[i] == 0u){
	    	huecos_ocupados[i] = 1u;
	        datos[i].distancia = distancia;
	        datos[i].angulo = angulo;
	        numero_huecos--;
	        return true;
	    }
	}
	return false;
}

//reserva hueco y hace transformacion a angulo
bool objetivo_guarda(float distancia, uint16_t angulo){
	return objetivo_guarda_g(distancia, transforma_g(angulo));
}

/////////////////////////////////
//libera hueco segun indice
bool objetivo_libera_indice(uint8_t indice){
	if (indice >= MAXIMO_OBJETIVOS){ return false;}
	if (huecos_ocupados[indice] == 0u){ return false;}

	huecos_ocupados[indice] = 0u;
	datos[indice].distancia = 0.0f;
	datos[indice].angulo = 0.0f;
    numero_huecos++;
    return true;
}
//libera hueco segun la posicion del angulo, en mi cabeza asi es como funcionaria
bool objetivo_libera_g(float angulo){
	uint8_t i = objetivo_indice_angulo_g(angulo);
    if (i == OBJETIVO_NO_ENCONTRADO){ return false;}
	return objetivo_libera_indice(i);
}
bool objetivo_libera(uint16_t angulo){
	return objetivo_libera_g(transforma_g(angulo));
}

/////////////////////////////////
//mira si el hueco esta ocupado
//muy importante para recorrer el vector y saltarse el hueco vacio
bool objetivo_hueco_usado(uint8_t indice){
	if (indice >= MAXIMO_OBJETIVOS){ return false; }
	    return (huecos_ocupados[indice] == 1u);
}

/////////////////////////////////
//devuelve la informacion de posicion
//si hubiese indice concreto se busca direcctamnete si no hay que ir uno a uno
Posicion* objetivo(uint8_t indice){
	if (indice >= MAXIMO_OBJETIVOS){ return NULL;}
	if (huecos_ocupados[indice] == 0u){ return NULL; }
	    return &datos[indice];
}

uint8_t objetivo_capacidad_total(void){
	return MAXIMO_OBJETIVOS;
}


/////////////////////////////////
//devuelve indice
uint8_t objetivo_indice_angulo_g(float angulo){
	for (uint8_t i = 0u; i < MAXIMO_OBJETIVOS; i++){
		if (huecos_ocupados[i] == 1u) { //esto se puede solo aqui, solo el posicion_pool.c ve las cosas static
			if (fabsf(datos[i].angulo - angulo) < margen_igualdad){
				return i;
			}
		}
	}
	return OBJETIVO_NO_ENCONTRADO;
}

uint8_t objetivo_indice_angulo(uint16_t angulo){
	return objetivo_indice_angulo_g(transforma_g(angulo));
}
