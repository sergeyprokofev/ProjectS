/*
 * bresenham.h
 * 
 * Copyright (c) 2013 by Oleg Trifonov <otrifonow@gmail.com>
 * 
 * Created on: Mar 20, 2013
 *
 * http://trolsoft.ru
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */


#ifndef _BRESENHAM_H_
#define _BRESENHAM_H_

#include <stdbool.h>

typedef struct bresenham_struct {
	uint8_t size;
	uint8_t value;
	int16_t error;
	uint8_t stepNumber;
} bresenham_struct;


void bresenham_init(struct bresenham_struct *st, uint8_t size) {
	st->size = size;
}

void bresenham_setValue(struct bresenham_struct *st, uint8_t val) {
	st->stepNumber = 0;
	st->value = val;
	st->error = st->size/2;
}

bool bresenham_getNext(struct bresenham_struct *st) {
	bool result;
    st->error -= st->value;
    if ( st->error < 0 ) {
    	st->error += st->size;
       result = true;
    } else {
       result = false;
    }
    if ( ++st->stepNumber >= st->size) {
    	st->stepNumber = 0;
    	st->error = st->size/2;
    }
    return result;
}


#endif // _BRESENHAM_H_
