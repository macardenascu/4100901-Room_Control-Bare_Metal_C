/**
 ******************************************************************************
 * @file           : room_control.c
 * @author         : Sam C
 * @brief          : Room control driver for STM32L476RGTx
 ******************************************************************************
 */
#include "room_control.h"

#include "gpio.h"    // Para controlar LEDs y leer el botón (aunque el botón es por EXTI)
#include "systick.h" // Para obtener ticks y manejar retardos/tiempos
#include "uart.h"    // Para enviar mensajes
#include "tim.h"     // Para controlar el PWM

void room_control_app_init(void)
{
    // Inicializar variables de estado si es necesario.
    // Por ejemplo, asegurar que los LEDs estén apagados al inicio

    // tim3_ch1_pwm_set_duty_cycle(50); // Establecer un duty cycle inicial para el PWM LED
}

void room_control_on_button_press(void)
{
    static uint32_t last_press_tick = 0;

    uint32_t now = systick_get_tick();
    if (now - last_press_tick < 300) return;  // Anti-rebote: ignora si fue hace <300ms

    last_press_tick = now;

    gpio_write_pin(GPIOA, EXTERNAL_LED_ONOFF_PIN, 1);  // Encender LED externo
    systick_delay_ms(3000);                            // Esperar 3 segundos (bloqueante)
    gpio_write_pin(GPIOA, EXTERNAL_LED_ONOFF_PIN, 0);  // Apagar LED
}

void room_control_on_uart_receive(char received_char)
{
    switch (received_char) {
        case 'h':
        case 'H':
            uart2_send_string("PWM LED al 100%\r\n");
            tim3_ch1_pwm_set_duty_cycle(100);
            break;

        case 'l':
        case 'L':
            uart2_send_string("PWM LED apagado\r\n");
            tim3_ch1_pwm_set_duty_cycle(0);
            break;

        case 't':
        case 'T':
            uart2_send_string("Toggle PWM LED\r\n");
            static uint8_t estado = 0;
            estado = !estado;
            tim3_ch1_pwm_set_duty_cycle(estado ? 100 : 0);
            break;

        default:
            uart2_send_string("Comando no reconocido\r\n");
            break;
    }
}

