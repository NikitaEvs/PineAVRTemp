/*
 * PineAVRTemp.c
 *
 * Author : Nikita Evsyukov
 */ 

/************************************************************************/
/*       FREQUENCY FOR PROTEUS - 1 MGHZ, IN REAL LIFE - 16 MGHZ         */
/*             CHANGE IT IN MAIN HEADER FILE (main.h)                   */
/*                CHECK PRESCALER FOR ADC INIT TOO                      */
/************************************************************************/

#include "main.h" // Please, be sure that frequency set right!

volatile unsigned long val = 0; // Variable for values from ADC 
volatile unsigned int count = 0; // Count of current measures

double refactor(int adcVal){
	return (double) (0.0488997555 * (adcVal - ZERO)); // Formula for conversion ADC to amperage
}

void sendUART(char data){
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0 = data;
}

void sendStr(const char *data){
	while(*data != 0x00){
		sendUART(*data);
		data++;
	}
	sendUART('\n');
}
void sendInt(int data){
	char str[10];
	sprintf(str, "%d", data);
	sendStr(str);
}
void sendDoub(double data){
	char str[100];
	dtostrf(data, 5, 3, str);
	sendStr(str);
}

void initUART(){
	/* Config pins for UART */
	DDRD &= (~(1<<pinRX));
	DDRD |= (1<<pinTX);
	/* Config UART speed */
	UBRR0L = (unsigned char) BAUDRATE;
	UBRR0H = (unsigned char) (BAUDRATE >> 8);
	/* Config UART */
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	UCSR0C = (1<<USBS0) | (3<<UCSZ00);
}

int getADC(){
	ADCSRA |= (1<<ADSC); // Start conversion
	while((ADCSRA & (1<<ADSC))); // Wait for ADSC return 0
	val += ADC;
	count++;
	return ADC;
}

void initADC(){
	ADMUX = 0x0;
	/* Init reference voltage at AVCC */
	ADMUX |= (1<<REFS0);
	/* Init input port at ADC0 */
	// MUX[3:0] = 0000
	/* ADC ON | Prescaler: 128 | Frequency: 125GGhz (be sure that prescaler set right! Check frequency of Atmega!) */
	ADCSRA |= (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);
	
	//ADCSRA |= (1<<ADATE) | (1<<ADIE); // Conversion with interrupt mode on (free running)
}

/* FOR DEBUG IN PROTEUS */

unsigned volatile long millis = 0;
unsigned long lastMillis = 0;

ISR (TIMER1_COMPA_vect){
	millis++;
}

/* Init debug timer on 1Ghz frequency */

void timer_init(){
	TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10); // CTC mode | Prescaler: 64 (CS11 = 1, CS10 = 1)
	TIMSK1 |= (1<<OCIE1A); // Interrupt when compare with OCR1A
	/* OCR1A = 250 */
	OCR1AH = 0b00000000;
	OCR1AL = 0b11111010;
	sei();
}

void debugMode(){
	cli();
	timer_init();
	lcd_initPort();
	lcd_init();
	lcd_setPos(0,1);
	lcd_sendStr("Start");
	DDRB |= (1<<PINB0);
	PORTB |= (1<<PINB0);
	_delay_ms(500);
	PORTB &= ~(1<<PINB0);
	_delay_ms(500);
	lcd_clear();
}

void debugData(){
	lcd_clear();
	lcd_setPos(0,1);
	lcd_sendInt(val/count);
	lcd_sendChar(' ');
	lcd_sendDoubleAccuracy(COUNT/(millis - lastMillis),1);
	lcd_sendChar(' ');
	lcd_sendDoubleAccuracy(1000.0/(millis - lastMillis), 1);
	lastMillis = millis;
	lcd_setPos(0,0);
	lcd_sendDouble(refactor(val/count));
}

/* END DEBUG FUNCTION */

int main(void) {	
	
	//debugMode();
	
	initADC();
	initUART();
	
    while (1) {
		getADC();
		if(count >= COUNT){
			double data = refactor(val/count);
			
			//debugData();
			
			//sendInt(val/count);
			sendInt((int)(1000*data));
			val = 0;
			count = 0;
		}
    }
}

