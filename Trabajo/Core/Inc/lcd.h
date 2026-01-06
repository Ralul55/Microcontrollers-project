#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define LCD_COLS 16
#define LCD_ROWS 2

void LCD_Init(void); //inicializa lcd
void LCD_Task(void); //funcion que actualiza lo q se ve en el display (llamada en el while)

void LCD_PrintfLine(uint8_t row, const char *text); //Printf para escribir una linea en display
void LCD_PrintfVar(uint8_t row, const char *formato, uint32_t value); //Printf para 1 valor numerico
void LCD_PrintfStr(uint8_t row, const char *formato, const char *value); //Printf para 1 string --No se si se usara pero por si aca

void LCD_WriteAt(uint8_t row, uint8_t col, const char *text);
void LCD_Clear(void);

#endif
