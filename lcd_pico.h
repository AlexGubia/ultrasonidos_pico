#ifndef LCD_PICOH
#define LCD_PICOH

#include "pico/stdlib.h"

/*
 -----------                   ----------
|   PICO    |                 |   LCD    |
|           |                 |          |
|     GPIO10|---------------->|D7        |
|     GPIO11|---------------->|D6        |
|     GPIO12|---------------->|D5        |
|     GPIO13|---------------->|D4        |
|           |                 |D3        |
|           |                 |D2        |
|           |                 |D1        |
|           |                 |D0        |
|           |                 |          |
|     GPIO14|---------------->|E         |
|           |         GND --->|RW        |
|     GPIO15|---------------->|RS        |
 -----------                   ----------
*/

#define PIN_LCD_D7     10          
#define PIN_LCD_D6     11           
#define PIN_LCD_D5     12         
#define PIN_LCD_D4     13                 
#define PIN_LCD_E      14                   
#define PIN_LCD_RS     15                   

// LCD info
#define lcd_LineOne     0x00                    // linea 1
#define lcd_LineTwo     0x40                    // linea 2

// LCD instrucciones
#define LCD_CLEAR           0b00000001          // código ASCII del carácter 'espacio'
#define LCD_HOME            0b00000010          // cursor en la posicion inicial
#define LCD_EM       		0b00000110          // movimiento del cursor al escribir
#define LCD_DOFF		    0b00001000          // apagar display
#define LCD_DON   		    0b00001100          // encender pantalla sin cursor parpadeando
#define LCD_RESET   		0b00110000          // reset del LCD
#define LCD_4BIT 			0b00101000          // 4-bit datos, 2 lienas, fuente 5 x 7 
#define LCD_SETCURSOR       0b10000000          // posicion del cursor

//LCD Funciones
void lcd_write_4(uint8_t);
void lcd_write_instruction_4d(uint8_t);
void lcd_write_character_4d(uint8_t);
void lcd_write_string_4d(uint8_t *);
void lcd_init_4d(void);

#endif
