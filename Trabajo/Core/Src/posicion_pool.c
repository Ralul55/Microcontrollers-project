#include "posicion_pool.h"
#include <math.h>
#include <stdbool.h> //para poder usar bool
#include "mapa.h"
#include "stm32f4xx_hal.h"


//gestion de la informacion estatica, solo 1 instancia
static Posicion datos[MAXIMO_OBJETIVOS];
static uint8_t huecos_ocupados[MAXIMO_OBJETIVOS]; //se encarga de anotar si el hueco esta libre (0) o ocupado (1)

static uint8_t numero_huecos = 0u;
static uint8_t numero_objetivos = 0u;
static uint8_t numero_abatidos = 0u;

//se necesita porque aparentemente comparar 2 floats no es trivial
static const float margen_igualdad = 0.01f;  // 0,01 grados

//1000 es 0, 2000 es 360
//es static, solo se puede usar desde aqui, si se necesita fuera pues se quita el static
static float transforma_g(uint16_t angulo){
	//comprobaciones que no se si deberian estar
		float resultado=(((float)angulo - 500.0) * 360.0 / 2000.0);
		if (angulo<500){return 0.0f;}
		if (angulo>2500){return 360.0f;}
		return resultado;
}

//Transformacion de grados a el valor entre 1000 (0º) y 2000 (360º) Se necesita  fuera asi que sin static
uint16_t transforma_a_entero(float angulo){
	float resultado = (500.0 + (angulo * 2000.0 / 360.0));

		if (resultado < 500.0f) resultado = 500.0f;
		if (resultado > 2500.0f) resultado = 2500.0f;

		return (uint16_t)(resultado + 0.5f); //+0.5f es para evitar truncamientos raros en el cast
}


void pool_init(void){
	for (uint8_t i = 0u; i < MAXIMO_OBJETIVOS; i++){
		huecos_ocupados[i] = 0u;
		datos[i].distancia = 0.0f;
		datos[i].angulo = 0.0f;
	    datos[i].marcado=0u;
	}
	numero_huecos = MAXIMO_OBJETIVOS;
	numero_objetivos = 0u;
	numero_abatidos = 0u;
}

/////////////////////////////////
// Para buscar un objetivo y fijarlo con el laser he creado esta función.
// La variable j es static para no perder la cuenta cuando se devuelva un objetivo de todos los posibles para en otra iteracion poder apuntar al siguiente de la lista.
Posicion* get_Objetivo(){
	static uint8_t j = 0u;

	    if (numero_huecos == MAXIMO_OBJETIVOS) {
	        return NULL; // no hay objetivos
	    }

	    if(datos[j].marcado==1u){datos[j].marcado=0u;} //se desmarca el objetivo anterior (solo se puede marcar un objetivo)
	    for (uint8_t k = 0u; k < MAXIMO_OBJETIVOS; k++) {
	        uint8_t idx = (uint8_t)((j + k) % MAXIMO_OBJETIVOS);
	        if (huecos_ocupados[idx] == 1u) {

	        	//si el objetivo encontrado esta abatido, si es asi, pasa al siguiente
	        	if (datos[idx].marcado==2u){continue;}

	            j = (uint8_t)((idx + 1u) % MAXIMO_OBJETIVOS);	 // siguiente para la próxima vez
	            datos[idx].marcado=1u; //se marca como objetivo
	            return &datos[idx];
	        }
	    }

	    return NULL;
}
//reserva hueco
bool objetivo_guarda_g(float distancia, float angulo){
	//si no hay hueco retorna falso
	if (numero_huecos == 0u){ return false; }

	for (uint8_t i = 0u; i < MAXIMO_OBJETIVOS; i++){
		if (huecos_ocupados[i] == 0u){
	    	huecos_ocupados[i] = 1u;
	        datos[i].distancia = distancia;
	        datos[i].angulo = angulo;
	        datos[i].marcado=0u;
	        numero_huecos--;
	        numero_objetivos++;
	        return true;
	    }
	}
	return false;
}

//reserva hueco y hace transformacion a angulo
bool objetivo_guarda(float distancia, uint16_t angulo){
	return objetivo_guarda_g(distancia, transforma_g(angulo));
}


//comprobacion de objetivo existente
bool objetivo_existente(uint16_t angulo_medio){
	uint8_t i = objetivo_indice_angulo(angulo_medio);
	if (i==OBJETIVO_NO_ENCONTRADO) return false;
	else return true;
}


/////////////////////////////////
//libera hueco segun indice
bool objetivo_libera_indice(uint8_t indice){
	if (indice >= MAXIMO_OBJETIVOS){ return false;}
	if (huecos_ocupados[indice] == 0u){ return false;}
		if (datos[indice].marcado==2u){numero_abatidos--;}
		else { numero_objetivos--; }

		huecos_ocupados[indice] = 0u;
		datos[indice].distancia = 0.0f;
		datos[indice].angulo = 0.0f;
		datos[indice].marcado=0u;
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


uint8_t objetivo_capacidad_total(void){uint8_t data = MAXIMO_OBJETIVOS; return data; }
uint8_t objetivo_objetivos_total(void){uint8_t data = numero_objetivos; return data; }
uint8_t objetivo_abatidos_total(void){uint8_t data = numero_abatidos; return data; }

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

/////////////////////////////////
//establecer indice como abatido
void objetivo_establecer_abatido(uint16_t angulo){
	uint8_t idx = objetivo_indice_angulo(angulo);
	    if (idx == OBJETIVO_NO_ENCONTRADO) return;
	datos[objetivo_indice_angulo(angulo)].marcado=2u;
	numero_abatidos++;
	numero_objetivos--;
}

void pool_reset(void){
	for (uint8_t i = 0u; i < MAXIMO_OBJETIVOS; i++){
		if (huecos_ocupados[i] == 1u) {
			(void)objetivo_libera_indice(i); //se hace cast a void, se puede hacer comprobacion
		}
	}
}


// Esta función combina todas las anteriores para hacer código funcional, la declaro aqui para dejar mas limpio el main.
void detectar_Objetivo(VL53L0X_RangingMeasurementData_t *Ranging, uint16_t angulo_actual){
	static float media_Grados = 0.0f;
	static float media_Distancia = 0.0f;
	static uint32_t t_last = 0;
	static float sumatorio_Grados = 0;
	static float sumatorio_Distancia = 0;
	static uint8_t numero_Objetivos = 0;
	static uint8_t flag_Objetivo_Detectado = 0;

	if (HAL_GetTick() - t_last < 50) return;
	t_last = HAL_GetTick();

	uint16_t distance_mm = Ranging->RangeMilliMeter;


	if (distance_mm <= distancia_actual){
	    flag_Objetivo_Detectado = 1;
	    sumatorio_Grados += angulo_actual;
	    sumatorio_Distancia += (float)distance_mm;
	    numero_Objetivos++;
	}

	else {
	    if (flag_Objetivo_Detectado == 1) {
	        if (numero_Objetivos > 0) { //Es una condicion redundante, pero por seguridad la he puesto
	            media_Grados = sumatorio_Grados / numero_Objetivos;
	            media_Distancia = sumatorio_Distancia / numero_Objetivos;

	            uint16_t ang_med = (uint16_t)(media_Grados + 0.5f);

	            if (!objetivo_existente(ang_med)){ //si el objetivo no esta guardado en lista, se almacena
	            	objetivo_guarda(media_Distancia, ang_med);
	            }
	        }

	        // reset
	        sumatorio_Grados = 0;
	        sumatorio_Distancia = 0;
	        numero_Objetivos = 0;
	        flag_Objetivo_Detectado = 0;
	    }
	    else {
	    	if (objetivo_existente(angulo_actual)){ //si el objetivo esta guardado en lista, se elimina
	    		//borra el objetivo del mapa
	            Posicion *p = objetivo(objetivo_indice_angulo(angulo_actual));
	            if (p != NULL) {
	            	mapa_borra_cuz(p);
	            }
	            objetivo_libera(angulo_actual);
	        }
	    }
	}
}

