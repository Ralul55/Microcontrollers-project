#include "lcd.h"
#include "liquidcrystal_i2c.h"
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>

static char display_buf[LCD_ROWS][LCD_COLS + 1]; //Almacena el texto que se va a escribir en display
static uint8_t cambio_pendt[LCD_ROWS]; // Indica si se ha realizado un cambio en alguna de las lineas del display
static uint32_t t_ultima_act = 0; //tiempo de ultima actuaizacion del display


// se encarga de hacer lineas de 16 bits +1 para \0
static void make_line(char out[LCD_COLS + 1], const char *in)
{
    for (int i = 0; i < LCD_COLS; i++) out[i] = ' ';
    out[LCD_COLS] = '\0';

    if (!in) return;

    for (int i = 0; i < LCD_COLS && in[i] != '\0'; i++)
        out[i] = in[i];
}


// funcion que imprime en el display la linea definitiva usando la libreria liquidcrystal
static void print_row(uint8_t row)
{
    HD44780_SetCursor(0, row);
    HD44780_PrintStr(display_buf[row]);
    cambio_pendt[row] = 0;
}


// inicializa el display usando libreria y lineas vacías
void LCD_Init(void)
{
    HD44780_Init(2);
    HD44780_Clear();

    for (uint8_t r = 0; r < LCD_ROWS; r++) {
        make_line(display_buf[r], "");
        cambio_pendt[r] = 1;                      // forzamos actualizacion del display
    }

    t_ultima_act = 0;
}


// funcion que llama a print_row para imprimir
void LCD_Task(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - t_ultima_act) < 2) return; //comprobamos que no han pasado menos de 2 ms antes de la ultima actuaizacion

    for (uint8_t r = 0; r < LCD_ROWS; r++) {
        if (cambio_pendt[r]) {   // si hay cambio pendiente, imprimimos
            print_row(r);
            t_ultima_act = now;
            return;
        }
    }
}


//funcion para imprimir texto (printf normal pero indicando linea)
void LCD_PrintfLine(uint8_t row, const char *text)
{
    if (row >= LCD_ROWS) return; //comprobamos que row es valido
    						     //si el texto introducido (cols) es mayor que 16 bits se truncará -> igual en todas las funciones printf
    char tmp[LCD_COLS + 1];
    make_line(tmp, text);

    if (strncmp(tmp, display_buf[row], LCD_COLS) == 0) return; //si ya está impreso (igual) descartamos

    memcpy(display_buf[row], tmp, LCD_COLS + 1); // si no, almacenamos en buffer
    cambio_pendt[row] = 1;
}


// funcion para imprimir texto y variable
void LCD_PrintfVar(uint8_t row, const char *formato, uint32_t value)
{
    if (row >= LCD_ROWS) return;

    char text[64];
    snprintf(text, sizeof(text), formato, (unsigned)value);

    LCD_PrintfLine(row, text);
}


// funcion para imprimir texto y string
void LCD_PrintfStr(uint8_t row, const char *formato, const char *value)
{
    if (row >= LCD_ROWS) return;

    char text[64];
    snprintf(text, sizeof(text), formato, value ? value : "");

    LCD_PrintfLine(row, text);
}


// funcion para sobrescribir en sitio especifico (no se si se usara)
void LCD_WriteAt(uint8_t row, uint8_t col, const char *text)
{
    if (row >= LCD_ROWS) return;
    if (col >= LCD_COLS) return;
    if (!text) return;

    uint8_t cambio = 0;
    uint8_t c = col;

    while (*text && c < LCD_COLS) {
        if (display_buf[row][c] != *text) {
        	display_buf[row][c] = *text;
            cambio = 1;
        }
        text++;
        c++;
    }

    if (cambio) cambio_pendt[row] = 1;
}

// funcion para vaciar display sin usar funcion clear de libreria (evitamos flicker y retardos innecesarios)
void LCD_Clear(void)
{
    for (uint8_t r = 0; r < LCD_ROWS; r++) {
        make_line(display_buf[r], "");
        cambio_pendt[r] = 1;
    }
}


















