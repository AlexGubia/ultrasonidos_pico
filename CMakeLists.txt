cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(ultrasonidos)

pico_sdk_init()

add_executable(ultrasonidos
	ultrasonidos.c
    lcd_pico
    lcd_pico.c
	)

#pico_enable_stdio_usb(_ 1)
#pico_enable_stdio_uart(_ 1)

# Agregar las funcionalidades de pico_stdlib
target_link_libraries(ultrasonidos pico_stdlib hardware_pwm hardware_spi)

# Crear mpa/bin/hex/uf2
pico_add_extra_outputs(ultrasonidos)

