#include "posicion_pool.h"
#include <math.h>
#include <stdbool.h> //para poder usar bool
#include "mapa.h"
#include <stdlib.h>
#include "stm32f4xx_hal.h"


//gestion de la informacion estatica, solo 1 instancia
static Posicion datos[MAXIMO_OBJETIVOS];
static uint8_t huecos_ocupados[MAXIMO_OBJETIVOS]; //se encarga de anotar si el hueco esta libre (0) o ocupado (1)

static uint8_t numero_huecos = 0u;
static uint8_t numero_objetivos = 0u;
static uint8_t numero_abatidos = 0u;

//se necesita porque aparentemente comparar 2 floats no es trivial
static const float margen_igualdad = 7.0f;  // 3 grados de error

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
uint16_t transforma_a_entero_radar(float angulo){
	float resultado = (500.0 + (angulo * 2000.0 / 360.0));

		if (resultado < 500.0f) resultado = 500.0f;
		if (resultado > 2500.0f) resultado = 2500.0f;

		return (uint16_t)(resultado + 0.5f); //+0.5f es para evitar truncamientos raros en el cast
}

//Transformacion de grados a el valor entre 950 (0º) y 1950 (90º) Se necesita  fuera asi que sin static
uint16_t transforma_a_entero_laser(float angulo){
	float resultado = (950.0 + (angulo * 1000.0 / 90.0));

		if (resultado < 950.0f) resultado = 950.0f;
		if (resultado > 1950.0f) resultado = 1950.0f;

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
	        //mapa_dibuja_cuz_g(objetivo(i));
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

		if (datos[indice].marcado != 2u && (numero_objetivos>0)){numero_objetivos--;}

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


uint8_t objetivo_capacidad_total(void){
	uint8_t data = MAXIMO_OBJETIVOS;
	if (data<0){data=0;}
	return data;
}

uint8_t objetivo_objetivos_total(void){
	uint8_t data = numero_objetivos;
	if (data<0){data=0;}
	return data;
}

uint8_t objetivo_abatidos_total(void){
	uint8_t data = numero_abatidos;
	if (data<0){data=0;}
	return data;
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

/////////////////////////////////
//establecer indice como abatido
void objetivo_establecer_abatido(uint16_t angulo){
	uint8_t idx = objetivo_indice_angulo(angulo);
	    if (idx == OBJETIVO_NO_ENCONTRADO) return;
	datos[objetivo_indice_angulo(angulo)].marcado=2u;
	numero_abatidos++;
	if (numero_objetivos>0){numero_objetivos--;}
}

void pool_reset(void){
	numero_abatidos=0;
	numero_objetivos=0;
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
	static uint32_t t_last = 0u;
	static float sumatorio_Grados = 0u;
	static float sumatorio_Distancia = 0u;
	static uint8_t numero_Objetivos = 0u;
	static uint8_t flag_Objetivo_Detectado = 0u;

	static uint16_t miss = 0u;
	static uint8_t flag_miss = 0u;
	static uint8_t flag_el_objetivo_es_miss = 0u;

	static Posicion* objetivo_faltante = NULL;

	const uint16_t umbral_miss =(uint16_t)(transforma_a_entero_radar(margen_igualdad) - transforma_a_entero_radar(0.0f));

	if (HAL_GetTick() - t_last < 50) return;
	t_last = HAL_GetTick();

	uint16_t distance_mm = Ranging->RangeMilliMeter;

	if (distance_mm <= distancia_actual){

		/*
		CREO QUE VA A HACER QUE SE DOBLE DEFINAN TODOS LOS OBJETIVOS

		//muy importante, si detecta algun objetivo limpia el miss
		flag_miss = 0u;
		objetivo_faltante = NULL;
		*/

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
				float ang_med_deg = transforma_g(ang_med);

				//SI HAY UN MISS Y ES EL OBJETIVO DETECTADO SE QUITA EL MISS
				if (flag_miss && (objetivo_faltante != NULL)) {
					//se comprueba que este dentro del margen, igual no hace falta
					if (fabsf(133u - ang_med_deg) < margen_igualdad) {
				        flag_miss = 0u;
				        objetivo_faltante = NULL;
				        miss = 0u;
				        flag_el_objetivo_es_miss=1;
				    }
				}

				//si el objetivo no esta guardado en lista ni era un miss, se almacena
				if (!flag_el_objetivo_es_miss&&!objetivo_existente(ang_med)){
					objetivo_guarda(media_Distancia, ang_med);

				}
			}

			// reset
			sumatorio_Grados = 0;
			sumatorio_Distancia = 0;
			numero_Objetivos = 0;
			flag_Objetivo_Detectado = 0;
			flag_el_objetivo_es_miss=0;

		}
		else {
		    uint8_t idx = objetivo_indice_angulo(angulo_actual);

		    //si hay un objetivo desaparecido (miss) comprueba la distancia que hay desde donde desaparecio a ahora
		    if (flag_miss && (objetivo_faltante != NULL)) {

		    	uint16_t dif;
		    	if (angulo_actual >= miss){dif = (uint16_t)(angulo_actual - miss);}
		    	else {dif = (uint16_t)(miss - angulo_actual);}

		    	//si la distancia sobrepasa el margen de igualdad sin encontrarlo se borra
		    	if (dif >= umbral_miss) {
		    		miss=0u;
		    		flag_miss=0u;
		    		mapa_borra_cuz(objetivo_faltante);
		    		objetivo_libera_g(objetivo_faltante->angulo);
		    		objetivo_faltante=NULL;
		    	}
		    }

		    // si deberia haber objetivo y no lo hay
		    if (idx != OBJETIVO_NO_ENCONTRADO) {

		    	Posicion *p = objetivo(idx);
    		   	//si no lo encuentra lo marca como desaparecido
		    	   if (p != NULL) {
		    		   	//comprueba que no este marcado ya el miss
						if (!flag_miss || (p != objetivo_faltante)) {
							objetivo_faltante = p;
							miss = angulo_actual;
							flag_miss = 1u;
						}
		    	   }
		    }

		}
	}

}
