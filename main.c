/*
 * Lab5.c
 * Control de 2 servos de 180° + LED con PWM manual por software
 * Autor: Arnulfo Díaz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// --- Declaraciones de funciones ---
void PWM_Servo_Init(void);
void PWM_Manual_Init(void);
void PWM_Manual_SetDuty(uint8_t duty);
uint16_t ADC_Read(uint8_t channel);
void ADC_Init(void);

// --- Variables globales ---
volatile uint8_t pwm_manual_duty = 0;  // Duty cycle del LED (0-255)
volatile uint8_t pwm_counter = 0;      // Contador para PWM manual

// =============================================
// CONFIGURACIÓN PWM PARA SERVOS (TIMER1 Y TIMER0)
// =============================================
void PWM_Servo_Init(void) {
    // Servo 1 (PB1 - OC1A): 50Hz (20ms período)
    DDRB |= (1 << PB1);                // PB1 como salida
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Modo 14 (Fast PWM), prescaler 8
    ICR1 = 19999;                      // TOP value para 20ms
    OCR1A = 1500;                      // Posición inicial (1.5ms)

    // Servo 2 (PD5 - OC0B): ~50Hz ajustado
    DDRD |= (1 << PD5);                // PD5 como salida
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // Fast PWM
    TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler 1024
    OCR0B = 23;                        // 1.5ms
}

// =============================================
// CONFIGURACIÓN ADC (3 CANALES)
// =============================================
void ADC_Init(void) {
    ADMUX = (1 << REFS0);              // Voltaje de referencia AVCC (5V)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Habilitar ADC, prescaler 128
}

uint16_t ADC_Read(uint8_t channel) {
    ADMUX = (1 << REFS0) | (channel & 0x07); // Seleccionar canal
    ADCSRA |= (1 << ADSC);             // Iniciar conversión
    while (ADCSRA & (1 << ADSC));      // Esperar fin de conversión
    return ADC;                        // Retornar valor de 10 bits (0-1023)
}

// =============================================
// PWM MANUAL PARA LED (TIMER2)
// =============================================
void PWM_Manual_Init(void) {
    DDRD |= (1 << PD6);                // PD6 como salida (LED)
    TCCR2A = 0;                        // Modo Normal
    TCCR2B = (1 << CS21);              // Prescaler 8 (16MHz/8 = 2MHz)
    TIMSK2 = (1 << TOIE2);             // Habilitar interrupción por overflow
}

void PWM_Manual_SetDuty(uint8_t duty) {
    pwm_manual_duty = duty;            // Actualizar duty cycle (0-255)
}

ISR(TIMER2_OVF_vect) {
    pwm_counter++;
    if (pwm_counter == 0) {
        PORTD |= (1 << PD6);           // Encender LED al inicio del ciclo
    }
    if (pwm_counter == pwm_manual_duty) {
        PORTD &= ~(1 << PD6);          // Apagar LED al alcanzar el duty
    }
}

// =============================================
// FUNCIÓN PRINCIPAL
// =============================================
int main(void) {
    // Inicializaciones
    ADC_Init();
    PWM_Servo_Init();                  // Configurar servos
    PWM_Manual_Init();                 // Configurar LED
    sei();                             // Habilitar interrupciones globales

    while (1) {
        // Leer potenciómetros y actualizar PWMs
        uint16_t adc0 = ADC_Read(0);   // Servo 1 (A0)
        uint16_t adc1 = ADC_Read(1);   // Servo 2 (A1)
        uint16_t adc2 = ADC_Read(2);   // LED (A2)

        // Mapear ADC a PWM (1ms-2ms para servos de 180°)
        OCR1A = (adc0 * 1000UL) / 1023 + 1000;  // Servo 1
        OCR0B = (adc1 * 1000UL) / 1023 + 1000;  // Servo 2
        PWM_Manual_SetDuty(adc2 >> 2); // LED: 10 bits ? 8 bits

        _delay_ms(20);                 // Estabilizar lecturas
    }
}