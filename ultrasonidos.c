#include <stdio.h>
#include <math.h>             //No debería de hacer falta por que SQRT se supone que está implementado en el pico/float.h o pico/double.h, pero el compilador saca warning
#include "pico/stdlib.h"
#include "pico/binary_info.h" //Para usar picotool y leer los binarios
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "lcd_pico.h"

#define BAUDRATE  	500000 		//Baudrate del SPI en HZ

#define PIN_INIT    3
#define PIN_INDEX   4

#define PIN_INT_1	23
#define PIN_INT_2 	1

#define PIN_PWM_A 	16
#define PIN_PWM_B 	17

#define PIN_MISO    8
#define PIN_CS	    9
#define PIN_SCK	    6
#define PIN_MOSI    7

#define WR_MDR0    0b10001000
#define WR_MDR1    0b10010000
#define WR_DTR	   0b10011000
#define LD_CNTR    0b11100000
#define RD_CNTR	   0b01100000

#define MDR0_DATA  0b01110100  //Sin cuadratura, Single-cycle counter, index en precarga del OTR, nivel del index alto
#define MDR1_DATA  0b00000110 //Contador 2 bytes, activado

uint8_t vInicialCont[] = {0xE8, 0x67};     // 59495d -> 0xE867
uint8_t mFallo[] = "-------";
uint8_t mInicio[] = "Alex Inza";

uint16_t temperatura = 20;

float v_aire(int16_t t);
static void conf_pwm(int pinA, int pinB);
static inline void cs_cont_1();
static inline void cs_cont_2();
static void envio_spi(uint8_t instruction, uint8_t *dato, uint8_t len);
static void envio_spi_ins(uint8_t instruction, uint8_t dato, uint8_t len);
static void lectura_cont(uint8_t instruction, uint8_t *lectura, uint8_t len);
static void conf_contadores();
static void irq_puls();
static void irq_sensor();

gpio_irq_callback_t int_fun_sensor_ptr = &irq_sensor;
gpio_irq_callback_t int_fun_pul_ptr = &irq_puls;

int main() {
	set_sys_clock_khz(100000, true); 	//Cambiamos la frecuencia de referencia de 125MHz a 100MHz
	stdio_init_all();					//Inicialización todo stdio

    //Pin con la señal init
	gpio_init(PIN_INIT);
	gpio_set_dir(PIN_INIT, GPIO_OUT);
    gpio_init(PIN_INDEX)
    gpio_set_dir(PIN_INDEX, GPIO_OUT);

	spi_init(spi0, BAUDRATE);
	gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
	gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

	//Pines SPI visibles para picotool por función
	bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

	gpio_init(PIN_CS);
	gpio_set_dir(PIN_CS, GPIO_OUT);

	//PIN_CS visible para picotool por nombre
	bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS"));

	//Configuración del PWM
	conf_pwm(PIN_PWM_A, PIN_PWM_B);

    //Inicio del LCD
    lcd_init_4d();
    lcd_write_string_4d(mInicio);

    //Preparamos los contadores para el primer ciclo
    conf_contadores();

    //Habilitación interrupción 1 (pulsador)
    gpio_set_irq_enabled_with_callback(PIN_INT_1, GPIO_IRQ_EDGE_FALL, true, int_fun_pul_ptr);

    while(1); //Bucle infinito a la espera de interrupciones

}


float v_aire(int16_t t) {
    return (331.4 + 0.6*t);
}

void conf_pwm(int pinA, int pinB) {
	gpio_set_function(pinA, GPIO_FUNC_PWM); //configura el pin seleccionado como salida pwm señal
	gpio_set_function(pinB, GPIO_FUNC_PWM); //configura el pin seleccionado como salida pwm referencia
	uint num_slice = pwm_gpio_to_slice_num(pinA); //busca que slice está conectada al pin de señal
	pwm_set_clkdiv_mode(num_slice, PWM_DIV_FREE_RUNNING); //modo free running
	pwm_set_phase_correct(num_slice, false); //sin correcion de fase, cuando llega a TOP vuelve a 0
	pwm_set_clkdiv_int_frac(num_slice, 1, 0); //configura el divisor del sys_clock para int=1 y frac=0
	pwm_set_output_polarity(num_slice, false, false); //configura la polaridad de los canales A y B a nula
	pwm_set_wrap(num_slice, 580); //Valor de TOP 580 para una señal de 172KHz
	pwm_set_both_levels(num_slice, 290, 290); //configura la referencia de los canales A y B para tener un ciclo de trabajo del 50%
	pwm_set_enabled(num_slice,  true);
}

static inline void cs_cont_1() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_cont_2() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void envio_spi(uint8_t instruction, uint8_t *dato, uint8_t len) {
	//MSB primero en el envio de datos
	//buffer instruccion + dato
	uint8_t buff[len+1];
	buff[0] = instruction;
	for(size_t i = 0; i < len; i++) buff[i+1] = *(dato+i);
	spi_write_blocking(spi0, buff, len+1);
	sleep_ms(10);
}

static void envio_spi_ins(uint8_t instruction, uint8_t dato, uint8_t len) {
    uint8_t buff[] = {instruction, dato};
	spi_write_blocking(spi0, buff, len);
	sleep_ms(10);
}

static void lectura_cont(uint8_t instruction, uint8_t *lectura, uint8_t len) {
	//Se recibe primero MSB de datos
    uint8_t ins = instruction;

    gpio_put(PIN_INDEX, 1);
    sleep_us(10);
    gpio_put(PIN_INDEX, 0);

	spi_write_blocking(spi0, &ins, 1);
	sleep_ms(10);
	spi_read_blocking(spi0, 0, lectura, len);
	sleep_ms(10);
}

static void conf_contadores() {
    //Configuracion de los contadores
    cs_cont_1();
    envio_spi_ins(WR_MDR0, MDR0_DATA, 1);
	envio_spi_ins(WR_MDR1, MDR1_DATA, 1);
	//Se cargan los contadores con el valor 65535-6040 = 59495
	envio_spi(WR_DTR, vInicialCont, 2);
    //------------------------
    cs_cont_2();
    envio_spi_ins(WR_MDR0, MDR0_DATA, 1);
	envio_spi_ins(WR_MDR1, MDR1_DATA, 1);
	envio_spi(WR_DTR, vInicialCont, 2);
}

//Subrutinas que ejecutar en las interrupciones
static void irq_puls() {
    sleep_ms(1);       //Como complemento al antirrebote
    gpio_set_irq_enabled_with_callback(PIN_INT_1, GPIO_IRQ_EDGE_FALL, false, int_fun_pul_ptr);

    //Señal de INIT para los sensores
    gpio_put(PIN_INIT, 1);
    sleep_ms(1);
    gpio_put(PIN_INIT, 0);

    //Deshabilitamos esta interrupción y configuramos la de los contadores
    gpio_set_irq_enabled_with_callback(PIN_INT_2, GPIO_IRQ_EDGE_FALL, true, int_fun_sensor_ptr);

}

static void irq_sensor() {
    uint8_t buffer[2];
    bool check = true;      //comprobación de que los valores son validos
    float t1 = 0;
    float distancia_B1 = 0;
    float t2 = 0;
    float distancia_B2 = 0;

    //Deshabilitamos la interrupción que ya estamos atendiendo
    gpio_set_irq_enabled_with_callback(PIN_INT_2, GPIO_IRQ_EDGE_FALL, false, int_fun_sensor_ptr);

    //Distancia B1 entre el sensor 1 y el obstáculo
    cs_cont_1();
    lectura_cont(RD_CNTR, buffer, 2);
    uint16_t pulsos_B1 = (buffer[0] << 8) + buffer[1] - 59495; //Hay que restarle el offset que se ha tenido en cuenta
    if (65535U - pulsos_B1) {
        t1 = pulsos_B1/172000.0;
        distancia_B1 = v_aire(20)/2*t1;
    } else check = false;

    //Distancia B2 entre el sensor 2 y el obstáculo
    cs_cont_2();

    lectura_cont(RD_CNTR, buffer, 2);
    uint16_t pulsos_B2 = (buffer[0] << 8) + buffer[1] - 59495; //Hay que restarle el offset que se ha tenido en cuenta
    if (65535U - pulsos_B2) {
        t2 = pulsos_B2/172000.0;
        distancia_B2 = v_aire(20)*(t2-t1/2);
    } else check = false;

    if (check) {
        //Cálculo de A
        float semiP = (distancia_B1 + distancia_B2 + 0.6)/2;
        float distancia_A = 2/0.6*(sqrt( semiP*(semiP-distancia_B1)*(semiP-distancia_B2)*(semiP-0.6) ));
        distancia_A = distancia_A*100;  //En centímetros como se pide
        //Valor de A en el lcd
        char mensaje[15];
        sprintf(mensaje, "%f", distancia_A);
        lcd_write_string_4d(mensaje);
    }
    else lcd_write_string_4d(mFallo);

    //Preparamos los contadores para próximos ciclos
    conf_contadores();

    //Configuración para habilitar de nuevo la pulsación e iniciar el ciclo de nuevo
    gpio_set_irq_enabled_with_callback(PIN_INT_1, GPIO_IRQ_EDGE_FALL, true, int_fun_pul_ptr);

}

/*
gpio_set_function(PIN_INT_1, PIO0_IRQ_0);
gpio_set_function(PIN_INT_1, PIO1_IRQ_0);

//Handlers con punteros a las funciones de las interrupciones
irq_handler_t int_fun_pul_ptr = &irq_puls;
irq_handler_t int_fun2_ptr = &interrupcion_2;

gpio_irq_callback_t int_fun_pul_ptr = &irq_puls;
gpio_irq_callback_t int_fun_sensor_ptr = &irq_sensor;

//Asignación de los handlers a las interrupciones
irq_set_exclusive_handler(7, int_fun1_ptr);			//Numero de 7 (PIO0_IRQ_0) la tabla de interrupciones al que asignar el handler
irq_set_exclusive_handler(9, int_fun2_ptr);			//Numero de 9 (PIO1_IRQ_0) la tabla de interrupciones al que asignar el handler

*/
