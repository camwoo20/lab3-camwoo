#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

/*--------------------Libraries---------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/*--------------------Variables---------------------------*/
char String[25];
volatile int currTime = 0;
volatile int sentSignal = 0;
volatile int riseTime = 0;
volatile int fallTime = 0;
volatile int pulseWidthTicks = 0;
volatile int receivedSignal = 0;
volatile int pulseWidthuS = 0;
volatile const float prescaler = 1024.0;
volatile const float clockFreqMhz = 16.0;
volatile float timePerTickuS = 0.0;
volatile int distanceCM = 0;
volatile int printDist = 0;
volatile int numOVF = 0;
volatile int currTickTRIG = 0;
volatile int discrete = 0;
volatile int i = 150;
volatile int prDutyCycle = 5;
const int maxClock = 65535;
/*-----------------------------------------------------------*/

void sendSignal() {
	sentSignal = 1;
	PORTD |= (1<<PORTD7); //set trigger pin to high
	_delay_ms(10);
	PORTD &= ~(1<<PORTD7); //set trigger pin to low
}

void calculateDistanceCM() {
	pulseWidthuS = (int) (pulseWidthTicks * timePerTickuS);
	distanceCM = (int) (pulseWidthuS / 58);
}

void calcOCR0AFromDistance() {
	OCR0A = 750000 / (-349 * distanceCM + 26512);
	if (discrete) {
		if (OCR0A <= 31) {
			OCR0A = 30;
			} else if (OCR0A <= 33) {
			OCR0A = 32;
			} else if (OCR0A <= 38) {
			OCR0A = 35;
			} else if (OCR0A <= 42) {
			OCR0A = 40;
			} else if (OCR0A <= 46) {
			OCR0A = 45;
			} else if (OCR0A <= 50) {
			OCR0A = 47;
			} else if (OCR0A <= 57) {
			OCR0A = 53;
			} else {
			OCR0A = 60;
		}
	}
}

int getDutyCycle() {
	prDutyCycle = 5;
	i = 150;
	while (i < 950) {
		if (ADC > i && ADC <= i + 80) {
			i = 150;
			return prDutyCycle;
		}
		i = i + 80;
		prDutyCycle = prDutyCycle + 5;
	}
}

void setBuzzerDutyCycle() {
	OCR2A = 255 * getDutyCycle() / 100;
}


void Initialize()
{
	cli();
	timePerTickuS = prescaler / clockFreqMhz;
	DDRD |= (1<<DDD6); // PD6 to output PWM frequency control buzzer
	DDRD |= (1<<DDD7); // PD7 to output Trigger
	DDRB &= ~(1<<DDB0); // PB0 to input (pin 8) Echo pin 
	DDRB &= ~(1<<DDB1); // PB1 to input (pin 9) (PCINT1) Toggle continuous or discrete 
	DDRB |= (1<<DDB3); // PB3 to output (pin 11) Volume control

	//prescale timer0 by 64 (8 bit)
	TCCR0B &= ~(1<<CS02);
	TCCR0B |= (1<<CS01);
	TCCR0B |= (1<<CS00);
	
	//prescale timer1 by 1024 --> 64 microseconds per tick --> min distance measurable is 1.1 cm (16 bit)
	TCCR1B |= (1<<CS10);
	TCCR1B &= ~(1<<CS11);
	TCCR1B |= (1<<CS12);

	//prescale timer 2 by 1
	TCCR2B &= ~(1<<CS22);
	TCCR2B &= ~(1<<CS21);
	TCCR2B |= (1<<CS20);
	
	//Set timer0 to PWM Phase correct mode (8 bit) 
	TCCR0B |= (1<<WGM02);
	TCCR0A &= ~(1<<WGM01);
	TCCR0A |= (1<<WGM00);
	
	//Set timer2 to fast PWM mode
	TCCR2B &= ~(1<<WGM22);
	TCCR2A |= (1<<WGM21);
	TCCR2A |= (1<<WGM20);
	
	//clear interrupt flag
	TIFR1 |= (1<<ICF1);
	//look for a rising edge first
	TCCR1B |= (1<<ICES1);
	
	//Turn on the power for ADC
	PRR &= ~(1<<PRADC);
	//Set reference voltage to AVcc
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1);
	//Scale ADC clock by 128
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);
	//Select ADC0 Pin
	ADMUX &= ~(1<<MUX0);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX3);
	//Enable autotriggering of ADC
	ADCSRA |= (1<<ADATE);
	//Enable free running mode
	ADCSRB &= ~(1<<ADTS0);
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2);
	//Disable the digital buffer on pin ADC0
	DIDR0 |= (1<<ADC0D);
	//Enable ADC
	ADCSRA |= (1<<ADEN);
	//Start conversion
	ADCSRA |= (1<<ADSC);
	
	//enable input capture interrupt
	TIMSK1 |= (1<<ICIE1);
	//Enable overflow interrupt on A
	TIMSK1 |= (1<<TOIE1);
	
	//Set timer0 to toggle OCR0A on compare match
	TCCR0A &= ~(1<<COM0A1);
	TCCR0A |= (1<<COM0A0);
	
	OCR0A = 32; //Find value to make it 440 Hz (Sets frequency). Will almost immediately be changed tho
	
	//Set timer2 to non-inverting mode
	TCCR2A |= (1<<COM2A1);
	TCCR2A &= ~(1<<COM2A0);
	
	OCR2A = 64; //start with a duty cycle of 1/4 so it's not loud. Will almost immediately be changed tho
	
	sei();
}

int main(void)
{
	Initialize();
	while(1)
	{
		if (PINB & (1<<PINB1)) {
			discrete = 1;
			} else if (!(PINB & (1<<PINB1))) {
			discrete = 0;
		}
		if (!sentSignal) {
			sendSignal();
		}
		if (printDist) {
			calculateDistanceCM();
			calcOCR0AFromDistance();
		}
		setBuzzerDutyCycle();
		prDutyCycle = 5;
	}
}

ISR(TIMER1_OVF_vect) {
	if (numOVF > 0) {
		sentSignal = 0;
		numOVF = 0;
	}
	numOVF = numOVF + 1;
}

ISR(TIMER1_CAPT_vect) {
	if (!receivedSignal) {
		receivedSignal = 1;
		TCNT1 = 0;
		TCCR1B ^= (1<<ICES1); //set to look for falling edge
		} else {
		pulseWidthTicks = ICR1;
		TCCR1B ^= (1<<ICES1); //look for rising edge
		receivedSignal = 0;
		sentSignal = 0;
		printDist = 1;
	}
}
