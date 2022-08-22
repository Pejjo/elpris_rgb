#include "include/driver_init.h"
#include "include/atmel_start_pins.h"
#include "include/uart.h"

#include <util/delay.h>
#include <avr/sleep.h>

uint8_t  Number_of_LEDS = 8; // Selects the number of RGB leds connected
uint8_t  ON             = 1;
uint8_t  OFF            = 0;
uint16_t address        = EEPROM_START; // Adress in the eeprom the color and intensity are stored

uint8_t portF_intflags = 0;
uint8_t status_flags   = 0; // Variable used to determine witch interrupts that have been triggered

typedef struct {
	uint8_t red; // RGB variables
	uint8_t green;
	uint8_t blue;
} rgb_t;

rgb_t leds[8];

void setup_interrupt_and_sleepmode(void);
void blink_selected(void);
void change_color(void);
void update_LEDS(uint8_t, uint8_t);
void SPI_Exchange8bit(uint8_t);

void get_stored_color_from_EE(void);
void store_color_to_EE(void);

ISR(PORTF_PORT_vect)
{
	portF_intflags = PORTF.INTFLAGS;

	if (portF_intflags & 0x40) { // Button 1 has been pressed and generated a low level interrupt
		status_flags |= 0x01;    // The status flag is set
		PORTF.INTFLAGS = 0x40;   // Interrupt flag cleared
		
		Led_0_toggle_level();
	}

}

// In main the
int main(void)
{
	
	uint8_t update;
	/* Initializes MCU, drivers and middleware */
	system_init();

	// Configure interrupts and sleepmode
	setup_interrupt_and_sleepmode();

	USART3_init();
	USART3_sendString("Type ON/OFF to control the LED.\r\n");
	
	// Make sure all LEDs are off
//	update_LEDS(OFF, Number_of_LEDS);

	// Get the stored color and intensity from eeprom
//	get_stored_color_from_EE();

	while (1) {
//		cli();
		if (status_flags == 0) // If no interrupts have been triggered, goto sleep
		{
			sei();
//			sleep_cpu(); // The device will remain in sleep until any of the buttons are pressed
		}                // to change color or intensity or when LUT0 output goes high.
		else {
			Check_flags(); // Check witch flags have been set
		}
		if (TCB0.INTFLAGS & TCB_CAPT_bm)
		{
			uint8_t cnt;
			char tmp[30];
			cnt=RingBuffer_GetCount(&inBuffer);
			TCB0.INTFLAGS=TCB_CAPT_bm;
			if (cnt==26)
			{
				update=0;
				uint8_t tmp;
				uint8_t * ledptr = leds;
				Led_0_set_level(1);
				USART3_sendString("S26");
				tmp=RingBuffer_Remove(&inBuffer);
				if (tmp==0x02)
					update=1;
				
				for (uint8_t x=1; x<25; x++)
				{
					USART3_sendString("*");
					*(ledptr++)=RingBuffer_Remove(&inBuffer);
				}
				tmp=RingBuffer_Remove(&inBuffer);
				if (tmp==0x03)
				{
					update++;
					USART3_sendString("E");
				}
				if (update==2)
				{
					USART3_sendString("Dne");
					write_to_leds(Number_of_LEDS);
				}
				
			}
			else
			{
				sprintf(&tmp, "C%d", cnt);
				USART3_sendString(&tmp);
				
				Led_0_set_level(0);
			}
			RingBuffer_InitBuffer(&inBuffer);
		}
	}
}

void Check_flags(void)
{
	switch (status_flags) {
	case 0x01:                            // Button 1 have been pressed to change the LED color
		while (Button_0_get_level() == 0) // Remain in the loop as long as the button is held low
		{
//			change_color();                  // Change to next color
//			update_LEDS(ON, Number_of_LEDS); // Write color to LEDs
//			_delay_ms(300);                  // Delay to allow user to observe the color
		}
//		store_color_to_EE();   // When button is released, the color is chosen and stored in eeprom
//		blink_selected();      // Selected color is displayed on the LEDs
		status_flags &= ~0x01; // Delete this status flag

		break;

	case 0x02:                            // Button 2 have been pressed to change the LED intensity
		while (0) //(Button_2_get_level() == 0) // Remain in the loop as long as the button is held low
		{
//			leds.intensity += 0x04;     // Change intensity
//			if (leds.intensity >= 0x30) // If intensity is set equal or higher than 0x30 it will be set back to 0x08
//			{ // This is done because the LEDs consume a lot of power, spesially when using white light
//				leds.intensity = 0x04;
//			}
//			update_LEDS(ON, Number_of_LEDS); // Write color to LEDs using new intensity
//			_delay_ms(300);                  // Delay to allow user to observe the intensity
		}
//		store_color_to_EE();   // When button is released, the intensity is chosen and stored in eeprom
//		blink_selected();      // Selected color is displayed on the LEDs
		status_flags &= ~0x02; // Delete this status flag

		break;

	case 0x03:
//		update_LEDS(ON, Number_of_LEDS); // Turn LEDs on
		status_flags &= ~0x03;           // Delete this status flag
		break;

	case 0x04:
//		update_LEDS(OFF, Number_of_LEDS); // Turn LEDs off
		status_flags &= ~0x04;            // Delete this status flag
		break;

	default:
//		update_LEDS(OFF, Number_of_LEDS); // Turn LEDs on
		status_flags = 0;                 // Delete all status flags
		break;
	}
}

void SPI_Exchange8bit(uint8_t data)
{
	// Composition of 24bit data:
	// G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0 B7 B6 B5 B4 B3 B2 B1 B0
	// Note: Follow the order of GRB to sent data and the high bit sent at first.
	// Clear the Write Collision flag, to allow writing
	SPI0.INTFLAGS = SPI0_INTFLAGS;

	// Reset TCA counter register to ensure the first rising edge of PWM is predictable
	TCA0.SINGLE.CNT = 0 /* Count: 0 */;

	// Start TCA
	TCA0.SINGLE.CTRLA = TCA_SPLIT_CLKSEL_DIV1_gc /* System Clock */
	                    | 1 << TCA_SPLIT_ENABLE_bp /* Module Enable: enabled */;

	// Start SPI by writing a byte to SPI data register
	SPI0.DATA = data;

	// Wait for transfer to complete
	while ((SPI0.INTFLAGS & SPI_RXCIF_bm) == 0) {
	}

	// Stop TCA
	TCA0.SINGLE.CTRLA = TCA_SPLIT_CLKSEL_DIV1_gc /* System Clock */
	                    | 0 << TCA_SPLIT_ENABLE_bp /* Module Enable: disabled */;
}

void write_to_leds(uint8_t x_leds)
{
	for (uint8_t n = 0; n < x_leds; n++) {
		// Transmit 24-bit RGB color data 8bit at a time using SPI
		SPI_Exchange8bit(leds[n].green); // GREEN
		SPI_Exchange8bit(leds[n].red);   // RED
		SPI_Exchange8bit(leds[n].blue);  // BLUE
	}
}

void setup_interrupt_and_sleepmode(void)
{

	PORTF_PIN6CTRL = 0x0B; // Enable low level interrupt on PF6 low(button 1)
	SLPCTRL.CTRLA = 1 << SLPCTRL_SEN_bp /* Sleep enable: enabled */
	                | SLPCTRL_SMODE_STDBY_gc /* Standby mode */;
}

void change_color(void)
{

}

void blink_selected(void)
{
}
