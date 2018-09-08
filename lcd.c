/*
 * lcd.c
 *
 * Author : Nikita Evsyukov
 */ 

#include "lcd.h"

/************************************************************************/
/*                          LCD CONNECTION                              */
/*----------------------------------------------------------------------*/
/*         LCD (4 bit mode)        |             Atmega                 */
/*----------------------------------------------------------------------*/
/*                D4              ->              PD4                   */
/*                D5              ->              PD5                   */
/*                D6              ->              PD6                   */
/*                D7              ->              PD7                   */
/*                E               ->              PD3                   */
/*                RW              ->              GND                   */
/*                RS              ->              PD2                   */
/*                VSS             ->              GND                   */
/************************************************************************/

/* Init all LCD connection ports */

void lcd_initPort(){
	DDRD |= (1<<E_PORT) | (1<<RS_PORT) | (1<<PIND4) | (1<<PIND5) | (1<<PIND6) | (1<<PIND7);
}

/* Send half of data byte (because using 4 bits PINs instead 8 PINs */

void lcd_sendHalfByte(unsigned char data){
	data <<= 4; // Using PD4-PD7
	E_HIGH; // Notify LCD about data byte
	_delay_us(50);
	CLEAR_DATA; // Clear data's pins
	PORTD |= data; // Push data
	E_LOW; // Off line
	_delay_us(50);
}

/* Send byte of data */
/*       mode:       */
/*  0 - data mode    */
/*  1 - command mode */

void lcd_sendByte(unsigned char data, unsigned char mode){
	if(mode == 1) {
		RS_LOW;
	} else {
		RS_HIGH;
	}
	lcd_sendHalfByte(data >> 4);
	lcd_sendHalfByte(data);
}

/* Config LCD mode */

void lcd_init(){
	/* All timing from datasheet */
	_delay_ms(15);
	lcd_sendHalfByte(LCD_FUNCTION_SET);
	_delay_ms(5);
	lcd_sendHalfByte(LCD_FUNCTION_SET);
	_delay_us(100);
	lcd_sendHalfByte(LCD_FUNCTION_SET);
	_delay_ms(1);
	lcd_sendHalfByte(LCD_FUNCTION_INIT);
	_delay_ms(1);
	lcd_sendByte(LCD_CONFIG_4BIT, 1);
	_delay_ms(1);
	lcd_sendByte(LCD_CONFIG_ON, 1);
	_delay_ms(1);
	lcd_sendByte(LCD_CONFIG_CURSOR, 1);
	_delay_ms(1);
}

/* Clear LCD display */

void lcd_clear(){
	lcd_sendByte(LCD_CLEAR, 1);
	_delay_us(1500);
}

/* Set cursor position */ 
/* y ^                 */
/*   |                 */
/*   |                 */
/*   +---------- >  x  */
/*  0                  */

void lcd_setPos(unsigned char posX, unsigned char posY){
	char RAM = (1-posY)*0x40 + posX;
	RAM |= LCD_CURSOR;
	lcd_sendByte(RAM, 1);
}

/* Send one char */ 

void lcd_sendChar(unsigned char data){
	lcd_sendByte(data, 0);
}

/* Send one string */

void lcd_sendStr(char data[]){
	for(wchar_t i = 0; data[i] != '\0'; i++){
		lcd_sendChar(data[i]);
	}
}

/* Send one int */

void lcd_sendInt(int data){
	char str[10];
	sprintf(str, "%d", data);
	lcd_sendStr(str);
}

/* Send one double with set count of number after decimal */

void lcd_sendDoubleAccuracy(double data, int numbAfterDecimal){
	char str[100];
	dtostrf(data, 5, numbAfterDecimal, str);
	lcd_sendStr(str);
}

/* Send one double */

void lcd_sendDouble(double data){
	char str[100];
	dtostrf(data, 5, 3, str);
	lcd_sendStr(str);
}