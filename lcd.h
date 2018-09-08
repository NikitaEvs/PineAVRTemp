/*
 * lcd.h
 *
 * Library for LCD (4 pin connection)
 *
 * PineAVRTemp
 *  Author: Nikita Evsyukov
 */ 


#ifndef LCD_H_
#define LCD_H_

#include "main.h"

/*  Macros for LCD  */

#define E_PORT PIND3
#define RS_PORT PIND2
#define E_HIGH PORTD |= (1<<E_PORT)
#define E_LOW PORTD &= ~(1<<E_PORT)
#define RS_HIGH PORTD |= (1<<RS_PORT)
#define RS_LOW PORTD &= ~(1<<RS_PORT)
#define CLEAR_DATA PORTD &= ~((1<<PIND4) | (1<<PIND5) | (1<<PIND6) | (1<<PIND7))
#define LCD_FUNCTION_SET 0b00000011
#define LCD_FUNCTION_INIT 0b00000010
/* DB4 = 0 (4bit mode) | DB3 = 1 (2 line mode) | DB5 = 1 (this command) | DB2 = 0 (normal size) */
#define LCD_CONFIG_4BIT 0b00101000
/* DB2 = 1 (display ON) | DB3 = 1 (this command) */
#define LCD_CONFIG_ON 0b00001100
/* DB2 = 1 (cursor moves to right) | DB1 = 1 (this command) */
#define LCD_CONFIG_CURSOR 0b00000110
#define LCD_CLEAR 0b00000001
#define LCD_CURSOR 0b10000000

/* End macros */

/* Functions */

void lcd_initPort(void);
void lcd_sendHalfByte(unsigned char data);
void lcd_sendByte(unsigned char data, unsigned char mode);
void lcd_init();
void lcd_clear();
void lcd_setPos(unsigned char posX, unsigned char posY);
void lcd_sendChar(unsigned char data);
void lcd_sendStr(char data[]);
void lcd_sendInt(int data);
void lcd_sendDoubleAccuracy(double data, int numbAfterDecimal);
void lcd_sendDouble(double data);

#endif /* LCD_H_ */