/*
 * main_project.c
 *
 * Created: 09/11/2025 13.05.06
 * Author : Tuukka Tikkakoski, 151000632, tuukka.tikkakoski@tuni.fi
 */ 

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>

#define ACTIVE_SWITCH_PIN PIND2
#define F_CPU 1000000UL // 1MHz clock
#define BAUD 2400UL
#define UBRR_VALUE (F_CPU / (16 * BAUD) - 1) // around ~25
#define MIN_HZ 50UL // The minimum Hz level for the sounder
#define MAX_HZ 1000UL // The maximum Hz level for the sounder
#define MAXIMUM_INPUT 1023UL // The maximum value for the ADC 10-bit result
#define PRESCALER_TIMER1 8UL
#define PRESCALER_TIMER0 256UL
#define DELAY 2UL // 2 seconds for the USART transmission
#define TARGET_OVERFLOWS ((DELAY * F_CPU) / (256UL * PRESCALER_TIMER0)) // Calculates the amount of overflows for the transmission to the virtual terminal.

volatile uint16_t current_freq = 250; // The current frequency for the sounder. (changes to something else when the program starts)
volatile uint16_t overflow_count = 0; // Overflow counter for timer0.
volatile uint8_t switch_state = 0; // Monitoring the state of the switch 0 = OFF, 1 = ON
volatile uint8_t led_brightness = 0;
volatile int8_t led_direction = 1;


// All the DDR and PORT initializations.
void initialize_pins() {
	DDRD &= ~(1 << DDD2); // Set PinD2 as input. (Switch)
	DDRD |= (1 << DDD3); // Set PinD3 as output. (LED)
	PORTD |= (1 << PORTD2); // Enable pull-up for PinD2.
	PORTC |= (1 << ADC2D); // Setting PORTC pin2 to 1 to minimize power consumption for the adc.
	DDRB |= (1 << DDB1); // Setting PORTB pin1 as output for the PWM.
}

// Checks if the switch is on.
uint8_t switch_is_on() {
	return !(PIND & (1 << ACTIVE_SWITCH_PIN));
}

// Used for initializing the adc conversion settings.
void set_adc() {
	
	DIDR0 |= (1 << ADC2D); // Disable C-port pin 2.
	
	ADMUX |= (1 << REFS0) | (1 << MUX1); // Setting AVCC as the reference voltage and ADC2 as the input channel for the potentiometer.
	
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // Set the prescaler to 8 so the AD clock is 1MHz / 8 = 125kHz
	
	PRR &= ~(1 << PRADC); // Enable AD-converter in Power Reduction Register.
	
	ADCSRA |= (1 << ADEN); // Enable AD-converter in ADCSRA.
}

// Used for setting sleep mode.
void activate_sleep_mode() {

	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set the mode to power-down.
	cli(); // Disable interrupts so the sleep mode will actually be activated.
	sleep_enable(); // Enabling sleep mode.
	sei(); // interrupts back on.
	sleep_cpu(); // Put the device in sleep mode.
	sleep_disable(); // Clears the sleep enable bit so the program can't enter sleep mode on it's own.
	
}

// Initializing the switch so that we can wake up from the sleep mode with an interrupt.
// Here, pin change interrupt is used.
void initialize_button_wake_up_interrupt() {
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT18);
}


void USART_Init() {
	UBRR0H = (unsigned char)(UBRR_VALUE>>8); // Setting upper bits.
	UBRR0L = (unsigned char)UBRR_VALUE; // Setting lower bits.
	UCSR0B = (1<<RXEN0) | (1<<TXEN0); // Enabling transmission and receiving.
	UCSR0B |= (1<<RXCIE0); // Enable receiver complete interrupt
}

// Transmits the bytes. 
void USART_transmit(unsigned char data) {
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

// Used for receiving the data from the virtual terminal.
unsigned char USART_Receive() {
	while (!(UCSR0A & (1<<RXC0))); // Wait until data is received
	return UDR0;                   // Return received byte
}

// Keeps sending the characters from the buffer until it's empty.
void USART_send_string(char* str) {
	while(*str) USART_transmit(*str++);
}

// Used for reading the ADC value.
uint16_t ADC_Read() {
	ADCSRA |= (1<<ADSC); // Start conversion.
	while(ADCSRA & (1<<ADSC)); // Wait until finished.
	return ADC;
}

// Used for led control with PWM.
void init_timer2_pwm() {
	TCCR2A = (1<<COM2B1) | (1<<WGM21) | (1<<WGM20); // Fast PWM, non-inverting on OC2B
	TCCR2B = (1<<CS21); // Prescaler = 8
	OCR2B = 0; // start with LED off
}

// Used for controlling the sounder frequency.
void init_timer1() {
	TCCR1A = (1<<COM1A0); // Toggle OC1A
	TCCR1B = (1<<WGM12) | (1<<CS11); // CTC mode and prescaler=8
}

// Used for sending the frequency to the terminal and setting the led brightness.
void init_timer0() {
	TCCR0A = 0;
	TCCR0B = (1<<CS02); // prescaler=256
	TIMSK0 = (1<<TOIE0); // enable overflow interrupt
}

// Used for incrementing the overflow count to control the led and USART transmission to the virtual terminal.
ISR(TIMER0_OVF_vect) {
	overflow_count++;
}

// This interrupt handler is used for setting the new frequency value when + or - is written to the terminal.
ISR(USART_RX_vect) {
	unsigned char received = UDR0; // Read the received byte.
	
	if (received == '+') {
		current_freq = current_freq + 10;
		char buffer[3];
		sprintf(buffer, "%c\r\n", received);
		USART_send_string(buffer); // Echo it back to the terminal.
	}
	if (received == '-') {
		current_freq = current_freq - 10;
		char buffer[3];
		sprintf(buffer, "%c\r\n", received);
		USART_send_string(buffer); // Echo it back to the terminal.
	}
	if (current_freq > MAX_HZ) {
		current_freq = MAX_HZ;
	}
	if (current_freq < MIN_HZ) {
		current_freq = MIN_HZ;
	}
	
	OCR1A = (F_CPU / (2UL * PRESCALER_TIMER1 * current_freq)) - 1; // Set the new value for the sounder.
}


// Used for wake up from sleep mode and setting the button state.
ISR(PCINT2_vect) {
	if (switch_is_on()) {
		switch_state = 1;  // switch ON
	} 
	else {
		switch_state = 0;  // switch OFF
	}
}


int main(void)
{
	initialize_pins();
	set_adc();
	initialize_button_wake_up_interrupt();
	init_timer0();
	init_timer1();
	init_timer2_pwm();
	USART_Init();
	
	sei(); // Enable global interrupts.
	
	// Setting the initial frequency for the sounder according to the potentiometer.
	uint16_t adc_val = ADC_Read();
	current_freq = MIN_HZ + ((adc_val * (MAX_HZ - MIN_HZ)) / MAXIMUM_INPUT);
	OCR1A = (F_CPU / (2UL * 8UL * current_freq)) - 1;
	
    while (1)
    {	
		// Constant check for sleep mode depending on the state of the switch.
		if (!switch_state) {
			activate_sleep_mode();
		}
		
		// This part makes the led change brightness by first going up in brightness and once it reaches the maximum value
		// it will switch the direction and lower the brightness. This makes the led blink smoothly.
		if (overflow_count % 1 == 0 && switch_state) {
			led_brightness += led_direction;
			if (led_brightness == 0 || led_brightness == 255) {
				led_direction = -led_direction; // switch direction for the led brightness so it dims or brightens 
			}
			OCR2B = led_brightness;
		}
		
		// Sending the frequency to the virtual terminal every ~2 seconds.
		if (overflow_count >= TARGET_OVERFLOWS) {
			overflow_count = 0;
			char buffer[32];
			sprintf(buffer, "Freq: %u Hz\r\n", current_freq);
			USART_send_string(buffer);
		}
    }
}

