#include "lcd_pico.h"


void lcd_init_4d(void) {
//Inicio the GPIOS
    gpio_init(PIN_LCD_D7); 
    //GPIO 10
    gpio_set_dir(PIN_LCD_D7, GPIO_OUT);
    gpio_init(PIN_LCD_D6); 
    //GPIO 11
    gpio_set_dir(PIN_LCD_D6, GPIO_OUT);
    gpio_init(PIN_LCD_D5); 
    //GPIO 12
    gpio_set_dir(PIN_LCD_D5, GPIO_OUT);
    gpio_init(PIN_LCD_D4); 
    //GPIO 13
    gpio_set_dir(PIN_LCD_D4, GPIO_OUT);
    gpio_init(PIN_LCD_E); 
    //GPIO 14
    gpio_set_dir(PIN_LCD_E, GPIO_OUT);
    gpio_init(PIN_LCD_RS); 
    //GPIO 15
    gpio_set_dir(PIN_LCD_RS, GPIO_OUT);

// delay encendido
    sleep_ms(100);                                 // 40 ms delay inicial


// Pines RS y E 
    gpio_put(PIN_LCD_RS, 0);                 // select del registro de instrucicones (RS bajo)
    gpio_put(PIN_LCD_E, 0);                  // E siempre a nivel bajo hasta que escribamos
// Reset LCD 
    lcd_write_4(LCD_RESET);                 // reset
    sleep_ms(10);                           // 4.1 ms delay (min)

    lcd_write_4(LCD_RESET);                 // reset 2
    sleep_us(200);                          // 100us delay (min)

    lcd_write_4(LCD_RESET);                 // reset 3
    sleep_us(200);							// no se cuanto


    lcd_write_4(LCD_4BIT);              	// modo 4-bit 
    sleep_us(80);                           // 40us delay (min)

// modo 4 bits
    lcd_write_instruction_4d(LCD_4BIT);     // set mode, lines, and font
    sleep_us(80);                           // 40us delay (min)

// apgar lcd
    lcd_write_instruction_4d(LCD_DOFF);     // apaga lcd
    sleep_us(80);                           // 40us delay (min)

// limpiar memoria lcd
    lcd_write_instruction_4d(LCD_CLEAR);    //limpia pantalla
    sleep_ms(4);                            // 1.64 ms delay (min)

// mover cursor
    lcd_write_instruction_4d(LCD_EM);       // modo de mover cursor
    sleep_us(80);                           // 40us delay (min)

// fin de init, ahora encendemos la pantalla

// encender pantalla
    lcd_write_instruction_4d(LCD_DON);         // pantalla ON
    sleep_us(80);                              // 40us delay (min)
}


void lcd_write_string_4d(uint8_t theString[]) {
    volatile int i = 0;                             // contador caracter 
    while (theString[i] != 0) {
        lcd_write_character_4d(theString[i]);
        i++;
        sleep_us(80);                              // 40 us delay (min)
    }
}


void lcd_write_character_4d(uint8_t theData) {
    gpio_put(PIN_LCD_RS, 1);                 // registro datos (RS alto)
    gpio_put(PIN_LCD_E, 0);                  // enable siempre bajo
    lcd_write_4(theData);                    // 4 bits altos
    lcd_write_4(theData << 4);               // 4 bits bajos
}


void lcd_write_instruction_4d(uint8_t theInstruction) {
    gpio_put(PIN_LCD_RS, 0);                // registro de instrucciones (RS bajo)
    gpio_put(PIN_LCD_E, 0);                 // enable siempre bajo hasta que escribamos
    lcd_write_4(theInstruction);            // 4 bits altos
    lcd_write_4(theInstruction << 4);       // 4 bits bajos
}


void lcd_write_4(uint8_t theByte) {
    gpio_put(PIN_LCD_D7, 0);                        // empezamos con el bit a 0
    if (theByte & 1<<7) gpio_put(PIN_LCD_D7, 1);    // si esta a 1, lo ponemos a 1

    gpio_put(PIN_LCD_D6, 0);                        // asi para todos
    if (theByte & 1<<6) gpio_put(PIN_LCD_D6, 1);

    gpio_put(PIN_LCD_D5, 0);
    if (theByte & 1<<5) gpio_put(PIN_LCD_D5, 1);

    gpio_put(PIN_LCD_D4, 0);
    if (theByte & 1<<4) gpio_put(PIN_LCD_D4, 1);
    
    // cambiar los datos (40ns)
    gpio_put(PIN_LCD_E, 1);                   // enable en alto para escribir
    sleep_us(1);                              // pequeña pausa
    gpio_put(PIN_LCD_E, 0);                   // enable en bajo para seguir haciendo cosas
    sleep_us(1);                              // pequeña pausa
}
