/*
 * uart0.h
 *
 *  Created on: Nov 2, 2019
 *      Author: mypc
 */

#ifndef UART0_H_
#define UART0_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart0();
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();

#endif /* UART0_H_ */
