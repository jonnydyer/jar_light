//
#ifndef GLOBAL_H
#define GLOBAL_H

// global AVRLIB defines
#include "avrlibdefs.h"
// global AVRLIB types definitions
#include "avrlibtypes.h"

#define SYSTICK_RATE 50
#define BTN_HOLD_TIME 3
#define TELEM_SLOW_DELAY 1.0
#define LED_UPDATE_DELAY 10
#define LED_FLASH_TICKS 17
// Debug defines
// MUST SUM TO 32!!
#define EEPR_MAX_ENTRIES 128      // 512/4
//#define EEPR_SAVE_ENABLE 1
//#define SPI_TLM_OUTPUT 1
//#define RAMP_PWM_MODE 1


#if defined(REV_A) || defined(REV_B)
// Defines

#define PWM_MASK (1<<PA7)

// Macros
#define TOGGLE_LED(x) PORTA = (PORTA & ~(x)) | (PORTA ^ (x))

#endif

#if defined(REV_A)
#define LED_MASK (1<<PA2)
#define ENABLE_BATT_MEAS DDRA |= (1<<PA3); PORTA |= (1<<PA3)
#define DISABLE_BATT_MEAS //DDRA &= ~(1<<PA3); PORTA &= ~(1<<PA3)

// BTN is not connected to PA4 on REV_A, but we have it hear so that it compiles.
// If BTN is needed, it can be accessed as the SCK line on the ISP header
#define BTN_MASK (1<<PA4)

// 20x differential input with ADC1(PA1) negative, ADC3 (PA3) positive; 1.1V ref
#define ADMUX_CURR (1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3) | (1<<MUX5) | (1<<REFS1)
#define ADMUX_VOLT (1<<MUX0) | (1<<MUX5)        // Internal reference
#define ADMUX_TEMP (1<<MUX1) | (1<<MUX5) | (1<<REFS1)

#elif defined(REV_B)
#define LED_MASK (1<<PA3)
#define BTN_MASK (1<<PA4)

#define ENABLE_BATT_MEAS 
#define DISABLE_BATT_MEAS 
// 20x differential input with ADC2(PA2) negative, ADC1 (PA1) positive; 1.1V ref
#define ADMUX_CURR (1<<MUX0) | (1<<MUX2) | (1<<MUX3) | (1<<REFS1)
#define ADMUX_VOLT (1<<MUX0) | (1<<MUX5)        // Internal reference
#define ADMUX_TEMP (1<<MUX1) | (1<<MUX5) | (1<<REFS1)

#else
#error "BOARD_REV must be one of [REV_A]"

#endif

#endif
