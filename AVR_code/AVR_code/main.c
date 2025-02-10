#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 1000000UL
#include <util/delay.h>
#define BAUD 2400UL
#define MYUBRR (F_CPU / 16UL / BAUD - 1UL)

/* Encoder signals */
volatile uint32_t encoder_interrupts = 0;
volatile uint8_t encoder_state = 0;
volatile uint8_t last_encoder_state = 0;

/* User options */
#define SET_SPEED 1
#define READ_SPEED_ONCE 2
#define READ_SPEED_CONT 3
#define GET_DUTY_CYCLE 4
#define SPEED_READ_TEST 5
#define TOGGLE_FINE_TUNING_ON_OFF 6
#define GET_FINE_TUNED_RPM_SETPOINT 7
#define GET_RPM_SETPOINT 8
#define RESET_AVR 222

volatile uint8_t user_is_inside_option = 0;
volatile uint8_t user_option = 0; // 0 = NO OPTION SELECTED

/* For speed reading test */
volatile uint8_t number_of_readings = 0;
volatile uint8_t number_of_readings_sent = 0;
volatile uint8_t waiting_for_next_speed = 0;
volatile uint8_t send_speed = 0;

volatile uint8_t fine_tuning_on_off = 0;

/* PI-controller */
volatile int32_t integral_FIX = 0;
volatile int32_t prev_error = 0;
volatile int32_t last_control_signal = 0;

/* RPM values */
volatile uint32_t rpm_setpoint = 0;
volatile uint32_t rpm_measured = 0;



/* ------------------------------------------------------------(      FUNCTIONS      )------------------------------------------------------------ */

void USART_init(unsigned int ubrr) {
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // Enable receiver, transmitter, and RX Complete interrupt
}

void USART_transmit(uint8_t send) {
	while( !(UCSR0A & (1 << UDRE0)) );
	UDR0 = send;
}


/* Function to enable AD converter for fine tuning of speed. */
void ADC_init(void) {
	PRR &= ~(1 << PRADC); // Disable power reduction for ADC

	ADMUX = (1 << REFS0); // Set AREF to internal 5 V
	ADMUX &= ~((1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0)); // Select ADC0

	ADCSRA = (1 << ADEN) | // Enable ADC
	(1 << ADATE) | // Enable auto trigger mode
	(1 << ADPS1) | (1 << ADPS0); // Set prescaler to 8 (ADC clock = 125 kHz)

	ADCSRB = 0; // Set trigger source to free-running mode

	DIDR0 = (1 << ADC0D); // Disable digital input on ADC0

	ADCSRA |= (1 << ADSC); // Start the first conversion
}


/* Function to convert ADC-value to RPM. */
int32_t ADC_to_RPM(int32_t adc_value) {
	int32_t steps_per_rpm = 40;
	
	if (adc_value >= 0 && adc_value <= 399) {
		// Map 0–399 to RPM between -10 and -1
		return -1 * ( (400 - adc_value) / steps_per_rpm );
	}
	else if (adc_value >= 400 && adc_value <= 623) {
		// Map 400–623 to RPM 0
		return 0;
	}
	else if(adc_value >= 624 && adc_value <= 1023){
		// Map 624–1023 to RPM between +10 and +1
		return (adc_value - 623) / steps_per_rpm;
	}
	
	return 0;
}


/* Function for fine tuning motor speed. */
uint32_t fine_tuned_RPM(uint32_t rpm_setpoint_value) {
	int32_t adc_value = ADC;
	int32_t rpm_adc = ADC_to_RPM(adc_value);
	int32_t rpm_sum = ((int32_t) rpm_setpoint_value) + rpm_adc;
	if(rpm_sum < 0L) {
		return 0;
	}
	else if(rpm_sum > 120L) {
		return 120;
	}
	else {
		return rpm_sum;
	}
}


/* Function to enable PWM output on PD5. */
void PWM_setup(void) {
	DDRD |= (1 << PD5); // Set PD5 (OC0B) as output
	TCCR0A |= (1 << WGM00); //Set counter0 (8-bit) mode to phase correct PWM
	TCCR0A |= (1 << COM0B1)| (1 <<COM0B0); // Inverting mode

	//Set prescaler to 1 for 1 MHz to get ~2 kHz.
	//PWM frequency for 8-bit counter = clk_io/(prescaler*510)
	TCCR0B |= (1 << CS00);

	//Set compare value (duty cycle)
	OCR0B = 0;
}


/* Function to enable interrupts on PD6 (encoder signal). */
void encoder_interrupts_init(void) {
	DDRD &= ~(1 << PD7);  // Set PD7 as input
	DDRD &= ~(1 << PD6);  // Set PD6 as input
	PORTD |= (1 << PD7);  // Enable pull-up on PD7
	PORTD |= (1 << PD6);  // Enable pull-up on PD6
	PCICR |= (1 << PCIE2); // Any change on any enabled PCINT23..16 pin will cause an interrupt.
	PCMSK2 |= (1 << PCINT23) | (1 << PCINT22); // Enable interrupts on PD7 and PD6
}


/* Function to initiate counter 1. */
void encoder_counter_init(void) {
	TCCR1B = (1 << CS11); // Prescaler 8
	TCNT1 = 0; // Reset counter
	TIMSK1 = (1 << TOIE1); // Enable overflow interrupt
}


/* Function to convert RPM to PWM duty cycle (0-255). */
uint32_t RPM_to_PWM(uint32_t speed_rpm) {
	return ( (1000UL * speed_rpm * 255UL) / 120UL / 1000UL );
}


uint8_t PI_controller(uint32_t set_rpm, uint32_t actual_rpm) {
	
	// Reset PI-controller if setpoint is 0.
	if(set_rpm == 0){
		integral_FIX = 0;
		prev_error = 0;
		last_control_signal = 0;
		return 0;
	}

	int32_t Kp_FIX = 0;
	int32_t Ki_FIX = 0;
	
	int32_t proportional = 0;
	int32_t integral = 0;
	int32_t control_signal = 0;
	
	int32_t min_control_signal = 0;
	int32_t max_control_signal = 255;
	
	int32_t set_pwm = RPM_to_PWM(set_rpm);
	int32_t actual_pwm = RPM_to_PWM(actual_rpm);
	
	int32_t current_error = set_pwm - actual_pwm;
	
	#define SMOOTHING_FACTOR 900L // 0.9 * 1000 fixed point. Adjust between 0 (no smoothing) and 1 (full smoothing)

	// Set parameters for PI-controller.
	if(set_rpm < 60UL){
		Kp_FIX = 50;//0.05 * 1000
		Ki_FIX = 7;//0.007 * 1000 fixed point
	}
	else if(set_rpm >= 60UL){
		Kp_FIX = 1000; //1 * 1000
		Ki_FIX = 40; //0.04 * 1000 fixed point
	}

	// Apply exponential filter to error.
	current_error = (int32_t) ( ( (SMOOTHING_FACTOR * prev_error) + ( (1000 - SMOOTHING_FACTOR) * current_error ) ) / 1000L );

	// Calculate proportional part and integral part.
	proportional = (Kp_FIX * current_error) / 1000L;
	integral_FIX = integral_FIX + (Ki_FIX * current_error);
	integral = integral_FIX / 1000L;
	
	// Cap integral part.
	if (integral < min_control_signal){
		integral = min_control_signal;
	}
	else if (integral > max_control_signal){
		integral = max_control_signal;
	}
	
	// Calculate control signal.
	control_signal = proportional + integral;
	
	// Cap control signal.
	if (control_signal < min_control_signal){
		control_signal = min_control_signal;
	}
	else if (control_signal > max_control_signal){
		control_signal = max_control_signal;
	}
	
	// Apply anti-windup
	if ( (control_signal == 255 && current_error > 0) || (control_signal == 0 && current_error < 0) ){
		integral_FIX = integral_FIX - (Ki_FIX * current_error);
	}
	
	prev_error = current_error;
	last_control_signal = control_signal;
	return control_signal;

}


/* Function to send data from AVR to PC. */
void send_to_PC(uint32_t measured_rpm_value, uint32_t rpm_tuned, uint32_t rpm_set_value, uint8_t fine_tuning, uint8_t duty_cycle){
	
	if(user_option == READ_SPEED_ONCE){
		USART_transmit( (uint8_t) measured_rpm_value );
		user_option = 0;
		user_is_inside_option = 0;
	}
	
	else if(user_option == READ_SPEED_CONT) {
		USART_transmit( (uint8_t) measured_rpm_value );
	}
	
	else if (user_option == GET_DUTY_CYCLE){
		USART_transmit(duty_cycle);
		user_option = 0;
		user_is_inside_option = 0;
	}
	
	else if(user_option == SPEED_READ_TEST) {
		if(send_speed == 1){
			if(number_of_readings_sent < number_of_readings) {
				USART_transmit( (uint8_t) measured_rpm_value );
				number_of_readings_sent += 1;
			}
			else {
				send_speed = 0;
				number_of_readings_sent = 0;
			}
		}
	}
	
	else if(user_option == TOGGLE_FINE_TUNING_ON_OFF){
		USART_transmit(fine_tuning);
		user_option = 0;
		user_is_inside_option = 0;
	}
	
	else if(user_option == GET_FINE_TUNED_RPM_SETPOINT){
		USART_transmit((uint8_t) rpm_tuned);
		user_option = 0;
		user_is_inside_option = 0;
	}
	else if(user_option == GET_RPM_SETPOINT){
		USART_transmit((uint8_t) rpm_set_value);
		user_option = 0;
		user_is_inside_option = 0;
	}
	
}

/* ----------------------------------------------------------(      FUNCTIONS END      )---------------------------------------------------------- */


/* ---------------------------------------------------(      INTERRUPT SERVICE ROUTINES      )--------------------------------------------------- */

/* Interrupt for USART receive completed */
ISR(USART_RX_vect) {
	
	uint8_t receivedData = UDR0;
	USART_transmit(receivedData);
	
	if(receivedData == RESET_AVR){
		
		user_option = 0;
		user_is_inside_option = 0;
		
		encoder_interrupts = 0;
		encoder_state = 0;
		last_encoder_state = 0;
		
		number_of_readings = 0;
		number_of_readings_sent = 0;
		waiting_for_next_speed = 0;
		send_speed = 0;
		
		fine_tuning_on_off = 0;
		
		integral_FIX = 0;
		prev_error = 0;
		last_control_signal = 0;

		rpm_setpoint = 0;
		rpm_measured = 0;
	}
	
	else if(user_is_inside_option == 0){
		
		if(receivedData == SET_SPEED){
			user_is_inside_option = 1;
			user_option = SET_SPEED;
		}
		else if(receivedData == READ_SPEED_ONCE){
			user_is_inside_option = 1;
			user_option = READ_SPEED_ONCE;
		}
		else if(receivedData == READ_SPEED_CONT){
			user_is_inside_option = 1;
			user_option = READ_SPEED_CONT;
		}
		else if(receivedData == GET_DUTY_CYCLE){
			user_is_inside_option = 1;
			user_option = GET_DUTY_CYCLE;
		}
		else if(receivedData == SPEED_READ_TEST){
			user_is_inside_option = 1;
			user_option = SPEED_READ_TEST;
			fine_tuning_on_off = 0;
		}
		else if(receivedData == TOGGLE_FINE_TUNING_ON_OFF){
			user_is_inside_option = 1;
			user_option = TOGGLE_FINE_TUNING_ON_OFF;
			fine_tuning_on_off ^= 1;
		}
		else if(receivedData == GET_FINE_TUNED_RPM_SETPOINT){
			user_is_inside_option = 1;
			user_option = GET_FINE_TUNED_RPM_SETPOINT;
		}
		else if(receivedData == GET_RPM_SETPOINT){
			user_is_inside_option = 1;
			user_option = GET_RPM_SETPOINT;
		}
	}
	
	else if(user_is_inside_option == 1){
		
		if(user_option == SET_SPEED){
			rpm_setpoint = (uint32_t) receivedData;
			user_option = 0;
			user_is_inside_option = 0;
		}
		else if(user_option == SPEED_READ_TEST) {
			if(waiting_for_next_speed == 0) {
				number_of_readings = receivedData;
				waiting_for_next_speed = 1;
			}
			else if (waiting_for_next_speed == 1) {
				rpm_setpoint = (uint32_t) receivedData;
				send_speed = 1;
			}
		}
	}
}


/* Interrupt for timer overflow. Calculating RPM. */
ISR(TIMER1_OVF_vect) {
	
	uint32_t temp1 =  ( 60 * F_CPU ) / ( 96 * 8 );
	temp1 = temp1 * 10000;
	temp1 = temp1 / 65536;
	temp1 = temp1 * encoder_interrupts;
	temp1 = temp1 / 10000;
	uint32_t rpm_raw = temp1;
	//Same as rpm_raw = ( encoder_interrupts * F_CPU * 60 ) / ( 96 * 65536 * 8 );
	
	if(encoder_interrupts > 214) { //214 interrupts correspond to 255 rpm which is larger than uint8_t
		rpm_measured = 255UL;
	}
	else {
		rpm_measured = rpm_raw;
	}
	encoder_interrupts = 0;
}


/* Interrupt for encoder signals. */
ISR(PCINT2_vect) {
	
	// Read the current state of PD7 and PD6
	uint8_t current_a = (PIND & (1 << PD7)) >> PD7;
	uint8_t current_b = (PIND & (1 << PD6)) >> PD6;

	encoder_state = (current_a << 1) | current_b; // Combine to 2-bit state

	if(last_encoder_state == encoder_state) {
		//Do nothing
	}
	else if ((last_encoder_state == 0b00 && encoder_state == 0b01) ||
	(last_encoder_state == 0b01 && encoder_state == 0b11) ||
	(last_encoder_state == 0b11 && encoder_state == 0b10) ||
	(last_encoder_state == 0b10 && encoder_state == 0b00)) {
		encoder_interrupts += 1;
		last_encoder_state = encoder_state;
	}
	else if ((last_encoder_state == 0b00 && encoder_state == 0b10) ||
	(last_encoder_state == 0b10 && encoder_state == 0b11) ||
	(last_encoder_state == 0b11 && encoder_state == 0b01) ||
	(last_encoder_state == 0b01 && encoder_state == 0b00)) {
		encoder_interrupts -= 1;
	}
}


/* -------------------------------------------------(      INTERRUPT SERVICE ROUTINES END      )------------------------------------------------- */


/* --------------------------------------------------------------(      MAIN      )-------------------------------------------------------------- */

int main(void) {
	
	USART_init(MYUBRR);
	encoder_interrupts_init();
	encoder_counter_init();
	PWM_setup();
	ADC_init();
	sei();
	
	uint32_t rpm_setpoint_value = 0;
	uint32_t rpm_tuned_setpoint = 0;
	uint32_t measured_rpm_value = 0;
	uint32_t rpm_setpoint_selected = 0;
	uint8_t fine_tuning = 0;
	
	
	while (1) {
		
		rpm_setpoint_value = rpm_setpoint;
		measured_rpm_value = rpm_measured;
		fine_tuning = fine_tuning_on_off;
		rpm_tuned_setpoint = fine_tuned_RPM(rpm_setpoint_value);
		
		if (fine_tuning == 0){
			rpm_setpoint_selected = rpm_setpoint_value;
		}
		else if (fine_tuning == 1){
			rpm_setpoint_selected = rpm_tuned_setpoint;
		}
		
		OCR0B = PI_controller(rpm_setpoint_selected, measured_rpm_value); //Set duty cycle
		
		send_to_PC(measured_rpm_value, rpm_tuned_setpoint, rpm_setpoint_value, fine_tuning, OCR0B);
		
		_delay_ms(20); // Sending data to PC and updating PI with f=50 Hz, T=20ms.
	}
	
}


/* ------------------------------------------------------------(      MAIN END      )------------------------------------------------------------ */


