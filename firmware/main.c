//
//  
//  
//

#include "global.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/cpufunc.h> 

#include <swTimer.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
// #include "usbdrv.h"
// #include "oddebug.h"        /* This is also an example for using debug macros */
// #include "requests.h"       /* The custom request numbers we use */

// Constants
#define KI 5
#define KD 0.0
#define KP 4
#define MAX_PWM 194
#define PWM_TOP 200
#define NUM_ADC_ACCUM 16
#define CNTRL_DEADBAND 32

#define VBATT_WARN_DN 320          // 1023 / 320 * 1.1 = 3.52V
#define VBATT_CUTOFF_DN 360        // 1023 / 360 * 1.1 = 3.13V

// Thresholds for battery SoC flashing:
// - 75% transition 3.90V... 1.1V / 3.9V * 1023 = 288 DN
// - 50% transition 3.75V... 1.1V / 3.75V * 1023 = 300 DN
// - 25% transition 3.70V... 1.1V / 3.70V * 1023 = 305 DN
#define VBATT_75_PERC 288
#define VBATT_50_PERC 300
#define VBATT_25_PERC 305

#define abs(x) ((x) > 0 ? (x) : (-x))

// Globals
const uint16_t g_set_pt_cycle[] = {
	0,
	5,
	10,
	25,
	50,
	100,
	200,
	300,
};

typedef enum event
{
    NO_EVENT,
    BTN_CLICK,
    BTN_HOLD,
    LOW_SOC,
    CNTRL_ERR
}event_t;

#if defined(REV_A)
uint8_t g_setting __attribute__ ((section (".noinit")));
#else
uint8_t g_setting=0;
#endif

event_t g_events;
sw_timer_t g_led_timer, g_led_flash_timer, g_button_timer, g_telem_slow_timer;
uint16_t g_set_pt;
volatile uint32_t g_systick=0;

#ifdef EEPR_SAVE_ENABLE
EEMEM uint32_t eeprom_entries[EEPR_MAX_ENTRIES];
#endif

// Protos
void init(void);
void init_pwm(void);
void init_adc(void);
void init_timer1(void);
void writeSPI(uint8_t *data, uint8_t len);
//void hadUsbReset(void);
event_t check_events(void);
uint16_t adc_read_samples(uint8_t admux, uint8_t n_samp, uint8_t adps, uint8_t settle);

uint8_t run_control(uint16_t adc_res, uint8_t reset);

int main (void) 
{
    event_t event = NO_EVENT;
    uint16_t adc_curr, adc_temp;
    uint16_t adc_volt = 0;
	uint8_t error = 0;
	uint8_t OCR0B_temp;
	uint8_t vbatt_warned = 0;
    uint8_t n_led_flashes = 0;
	
#ifdef SPI_TLM_OUTPUT
	uint8_t spi_packet[8] = {0,0,0,0,0,0, 0xaa, 0xaa};
#endif

#ifdef EEPR_SAVE_ENABLE
	uint8_t eeprom_ind = 0;
	uint32_t eeprom_entry;
#endif

#ifdef RAMP_PWM_MODE
	uint8_t i;
#endif

	cli();
	GIFR |= (1<<INTF0);
	
	init();

#if defined(REV_A)
	g_set_pt = g_set_pt_cycle[g_setting];
	g_setting++;
	if(g_setting >= sizeof(g_set_pt_cycle)/2)
	{
		g_setting = 0;
	}
#else
    g_setting = 0;
    g_set_pt = 0;
#endif

	while(1)
	{
        wdt_reset();
		// We are here because we woke up - first thing to do is turn off LED.
        PORTA &= ~LED_MASK;

		if(swTimer_expired(&g_led_timer))
		{
            if(adc_volt < VBATT_75_PERC)
                n_led_flashes = 4;
            else if(adc_volt < VBATT_50_PERC)
                n_led_flashes = 3;
            else if(adc_volt < VBATT_25_PERC)
                n_led_flashes = 2;
            else
                n_led_flashes = 1;

            swTimer_start(&g_led_flash_timer);
			swTimer_restart(&g_led_timer);
		}

        if(swTimer_expired(&g_led_flash_timer))
        {
            if(n_led_flashes > 0)
            {
	            PORTA |= LED_MASK;
                swTimer_restart(&g_led_flash_timer);
                n_led_flashes--;
            }
        }
        
        event = check_events();
        
        // Handle button clicks
        if(event == BTN_CLICK)
        {
            g_setting++;
            if(g_setting >= sizeof(g_set_pt_cycle)/2){
                g_setting = 0;
                run_control(0, 1);
				vbatt_warned = 0;
            }
            g_set_pt = g_set_pt_cycle[g_setting];
        }
        else if(event == BTN_HOLD)
        {
            // Spin until WDT reset...
			PORTA |= LED_MASK;
			_delay_ms(50);
			PORTA &= ~LED_MASK;
			_delay_ms(250);
			PORTA |= LED_MASK;
			_delay_ms(50);
			PORTA &= ~LED_MASK;
			_delay_ms(250);
			PORTA |= LED_MASK;
			_delay_ms(50);
			PORTA &= ~LED_MASK;
			_delay_ms(250);
            while(1){};
        }
        else
        {
			// We read the channels in this order for a reason - 
			if(swTimer_expired(&g_telem_slow_timer)){
				adc_temp = adc_read_samples(ADMUX_TEMP, 1, (1<<ADPS2), 1);   // clk_adc/16
				adc_curr = adc_read_samples(ADMUX_CURR, NUM_ADC_ACCUM, (1<<ADPS2), 0);   // clk_adc/16
				adc_volt = adc_read_samples(ADMUX_VOLT, 1, (1<<ADPS2), 1);   // clk_adc/16
				// Call one more time with current channel settings to allow everything to 
				// re-settle with the 1.1V reference
				adc_read_samples(ADMUX_CURR, 1, (1<<ADPS2), 1);   // clk_adc/16
				
			
#ifdef EEPR_SAVE_ENABLE
				// write eeprom
				if(eeprom_ind < EEPR_MAX_ENTRIES)
				{
					eeprom_entry =  (((uint32_t)(adc_curr >> 2)) << 20) & 0xFFF00000;
					eeprom_entry += (((uint32_t)adc_volt) << 10) & 0x000FFC00;
					eeprom_entry += (((uint32_t)adc_temp)) & 0x000003FF;
					eeprom_write_dword(&eeprom_entries[eeprom_ind++], eeprom_entry);
				}
#endif

#ifdef SPI_TLM_OUTPUT
				
				//Write out to SPI port
				((uint16_t*)spi_packet)[0] = adc_curr;
				((uint16_t *)spi_packet)[1] = adc_volt;
				((uint16_t *)spi_packet)[2] = adc_temp;
				((uint16_t *)spi_packet)[3] = OCR0B;
				writeSPI(spi_packet, 8);
#endif

#ifdef RAMP_PWM_MODE
				i++;
				if(i & 0x01){
					if(OCR0B < MAX_PWM)
					{
						//OCR0B += 1;
					}
					else
					{
						OCR0B = 0;
					}
				}
#endif
				if((adc_volt >= VBATT_WARN_DN) && (adc_volt < VBATT_CUTOFF_DN) && (vbatt_warned == 0) && (g_setting > 0))
				{
					// Warn user by blinking 3 times...
					OCR0B_temp = OCR0B;
					OCR0B = 0;
					_delay_ms(250);
					OCR0B = OCR0B_temp;
					_delay_ms(250);
					wdt_reset();
					OCR0B = 0;
					_delay_ms(250);
					OCR0B = OCR0B_temp;
					_delay_ms(250);
					wdt_reset();
					OCR0B = 0;
					_delay_ms(250);
					OCR0B = OCR0B_temp;
					wdt_reset();
					vbatt_warned = 1;
				}
				else if((adc_volt >= VBATT_CUTOFF_DN) && (g_setting > 0))
				{
					g_setting = 0;
	                run_control(0, 1);
					vbatt_warned = 0;
	            	g_set_pt = g_set_pt_cycle[g_setting];
				}
				swTimer_start(&g_telem_slow_timer);
			}
			else{
				adc_curr = adc_read_samples(ADMUX_CURR, NUM_ADC_ACCUM, (1<<ADPS2), 0);   // clk_adc/16
			}
            
#ifndef RAMP_PWM_MODE
            // Run Control
            error = run_control(adc_curr, 0);
			if(error){
				g_setting = 0;
                run_control(0, 1);
				vbatt_warned = 0;
            	g_set_pt = g_set_pt_cycle[g_setting];
			}
			//OCR0B = 10;
#endif

        }
        // Pole USB
		//usbPoll();
        
		wdt_reset();
        //PORTA &= ~LED_MASK;
        sleep_mode();
	}
	return 0;
}

uint8_t run_control(uint16_t adc_res, uint8_t reset)
{
	int32_t err, cmd;
	static int32_t i_err = 0;
	/* Scaling:
	 * 1 Raw ADC DN = 1.1V / 20x (gain) / 1024 = .05371 mA
	 * We are oversampling 16x -> 1 OS'd DN = 0.05371 / 16 = 3.35693 uA
	 */
	if(reset){
        i_err = 0;
        OCR0B = 0;
        return 0;
    }
	
	// Now compute err in mA
	err = (int32_t)(0.1 / (.05371 / NUM_ADC_ACCUM) ) * g_set_pt - adc_res;
	if(err > 0)
	{
        i_err += err;
        // cmd = (uint8_t)(KI * i_err - KD * (err - err_last));
        cmd = (uint8_t)(KI * i_err / 1000);
        if(cmd > MAX_PWM)
        {
            OCR0B = 0;
			i_err = 0;
			return 1;
        }
        else
        {
            OCR0B = cmd;
        }
	}
	
	return 0;
}

/*
usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;
static uchar    dataBuffer[4];  // buffer must stay valid when usbFunctionSetup returns

    if(rq->bRequest == CUSTOM_RQ_ECHO){ // echo -- used for reliability tests
        dataBuffer[0] = rq->wValue.bytes[0];
        dataBuffer[1] = rq->wValue.bytes[1];
        dataBuffer[2] = rq->wIndex.bytes[0];
        dataBuffer[3] = rq->wIndex.bytes[1];
        usbMsgPtr = dataBuffer;         // tell the driver which data to return
        return 4;
    }else if(rq->bRequest == CUSTOM_RQ_SET_STATUS){
        if(rq->wValue.bytes[0] & 1){    //set LED
            PORTA |= LED_MASK;
        }else{                          // clear LED
            PORTA &= ~LED_MASK;
        }
    }else if(rq->bRequest == CUSTOM_RQ_GET_STATUS){
        dataBuffer[0] = ((PORTA & LED_MASK) != 0);
        usbMsgPtr = dataBuffer;         // tell the driver which data to return
        return 1;                       // tell the driver to send 1 byte
    }
    return 0;   //default for not implemented requests: return no data back to host
}
*/

void init(void)
{
	// NOTE: We don't call swTimer_init() because it will reset our clock settings.
	/*
	// If we stored OSCCAL with our bootloader, load it up to calibrate oscillator
	unsigned char stored_osc_calibration = pgm_read_byte(BOOTLOADER_ADDRESS - TINYVECTOR_OSCCAL_OFFSET);
	  if (stored_osc_calibration != 0xFF) {
	    OSCCAL=stored_osc_calibration;
	    _NOP();
	  }
	*/
	
	init_timer1();
	init_pwm();
	init_adc();
    wdt_enable(WDTO_1S);
	//wdt_disable();

	// TODO: PA4 is for debugging USB interrupts and should be removed at some point
	DDRA = LED_MASK | PWM_MASK | (1<<PA5);
	PORTA = BTN_MASK | LED_MASK;
	
    PCMSK0 = BTN_MASK;
    GIMSK |= (1<<PCIE0);
	
	DISABLE_BATT_MEAS;
	
	/*
	cli();
	usbInit();    // Initialize INT settings after reconnect
	
	sei();
	usbDeviceDisconnect();  // do this while interrupts are disabled
	_delay_ms(300);
  	usbDeviceConnect();
	*/

    sei();
	
	swTimer_set(&g_led_timer, LED_UPDATE_DELAY * SYSTICK_RATE);
	swTimer_start(&g_led_timer);

	swTimer_set(&g_button_timer, SYSTICK_RATE * BTN_HOLD_TIME);
    swTimer_set(&g_led_flash_timer, LED_FLASH_TICKS);
	
	swTimer_set(&g_telem_slow_timer, TELEM_SLOW_DELAY * SYSTICK_RATE);
	swTimer_start(&g_telem_slow_timer);
}

void writeSPI(uint8_t *data, uint8_t len)
{
	uint8_t DDRA_old = DDRA;
	uint8_t PORTA_old = PORTA;
	uint8_t PCMSK0_old = PCMSK0;
	uint8_t USICR_clock = (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC);  // Three wire mode, software strobe clock
	uint8_t i;
	
	PCMSK0 = 0x00;
	DDRA |= (1<<PA4) | (1<<PA5) | (1<<PA6);    // DO and USCK
	
	//PULSE CS high...
	PORTA |= (1<<PA6);
	_delay_us(100);
	PORTA &= ~(1<<PA6);
	
	for(i=0;i<len+1;i++){
		USIDR = data[i];
		USISR |= (1<<USIOIF);
		while(!(USISR & (1<<USIOIF))){
			USICR = USICR_clock;
			_delay_us(10);
		}
	}
	
	//PULSE CS high...
	PORTA |= (1<<PA6);
	_delay_us(100);
	
	DDRA = DDRA_old;
	PORTA = PORTA_old;
	PCMSK0 = PCMSK0_old;
}

void init_timer1(void)
{
	TCCR1A = 0;
	TCCR1B = (1<<CS11) | (1<<WGM12);          //CTC, clk/8 gives 1us/cnt @ 8MHz
	TIMSK1 |= (1<<OCIE1A);		  			  // Enable interrupt
    OCR1A = (uint16_t)(F_CPU / 8 / SYSTICK_RATE);
	TCNT1 = 0;
}

void init_adc(void)
{
	// ADC enabled
	ADCSRA = (1<<ADEN);
	DIDR0 |= (1<<ADC1D) | (1<<ADC2D);    // Disable digital input buffer to save a little power
}

void init_pwm(void)
{
	TCCR0A = (1<<COM0B1) | (1<<WGM00) | (1<<WGM01); // Non-inverting fast PWM with Top at OCRA
	//TCCR0A = (1<<COM0B1) | (1<<WGM00); // Non-inverting phase-correct PWM with Top at OCRA
	TCCR0B = (1<<CS00) | (1<<WGM02);  // No pre-scaling
	OCR0A = PWM_TOP;
	OCR0B = 0;
}

/*
void hadUsbReset(void)
{
    calibrateOscillatorASM();
    //TOGGLE_LED(LED_MASK);
}*/

event_t check_events(void)
{
    if(g_events == BTN_CLICK)
    {
        g_events = NO_EVENT;
        return BTN_CLICK;
    }

    if(swTimer_expired(&g_button_timer))
    {
        swTimer_stop(&g_button_timer);
        return BTN_HOLD;
    }

    return NO_EVENT;
}

uint16_t adc_read_samples(uint8_t admux, uint8_t n_samp, uint8_t adps, uint8_t settle)
{
    uint8_t i;
    uint16_t res = 0;
    ADMUX = admux;
    ADCSRA = (1<<ADEN) | (adps & 0x07);
	if(settle)
	{
		for(i=0; i<settle; i++)
			_delay_ms(1);
	}
		
    for(i=0; i<n_samp; i++)
    {
        sbi(ADCSRA, ADSC);
        while(bit_is_set(ADCSRA, ADSC)){};
        res += ADC;
        //ADCSRA |= (1<<ADIF);  // CLear conversion complete flag
    }
    return res;
}

ISR(TIM1_COMPA_vect)
{
	timerCallback();
}

ISR(PCINT0_vect)
{
    static uint8_t btn_state;
    btn_state = PINA & BTN_MASK;

    if(btn_state > 0)           // Button released
    {
        // First check if we held for longer than button hold timeout...
        if(swTimer_expired(&g_button_timer))
            return;
        
        // If not, this was a "short" click
        g_events = BTN_CLICK;
        swTimer_stop(&g_button_timer);
        return;
    }
    else                        // Button pressed
    {
        swTimer_set(&g_button_timer, SYSTICK_RATE * BTN_HOLD_TIME);
        swTimer_start(&g_button_timer);
        return;
    }

}
