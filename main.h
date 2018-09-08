/*
 * main.h
 *
 * PineAVRTemp
 *
 * 
 *  Author: Nikita Evsyukov
 */ 


#ifndef MAIN_H_
#define MAIN_H_

/************************************************************************/
/*       FREQUENCY FOR PROTEUS - 1 MGHZ, IN REAL LIFE - 16 MGHZ          */
/************************************************************************/

#ifndef F_CPU
#define F_CPU 16000000UL 
#endif

/* Linking libraries */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#include "lcd.h"

/*     Macros       */

/* UART pins */
#define pinRX PIND0
#define pinTX PIND1
/* UART speed config */
#define BAUD 256000
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)

#define COUNT 1 // Count of measures

/* End macros */

#endif /* MAIN_H_ */