/*
 * wiring.h
 *
 *  Created on: 13/02/2016
 *      Author: xottam
 */

#ifndef WIRING_H_
#define WIRING_H_

/*
  wiring.h - Partial implementation of the Wiring API for the ATmega8.
  Part of Arduino - http://www.arduino.cc/
  Copyright (c) 2005-2006 David A. Mellis
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
  $Id: wiring.h 590 2009-05-24 15:12:19Z dmellis $
*/



#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f10x_type.h"

//#define PROGMEM const
#define pgm_read_byte(addr) (*( signed char *)(addr))
#define pgm_read_word(addr) (((*((unsigned char *)addr + 1)) << 8) + (*((unsigned char *)addr)))



#define pgm_read_word_near(addr) pgm_read_word(addr)





/**#ifdef abs
#undef abs
#endif
*/
#define abs(x) (((x) > 0) ? (x) : -(x))

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))



#ifdef __cplusplus
} // extern "C"
#endif



#endif /* WIRING_H_ */
