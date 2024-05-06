/*
 * x-jsn.h
 *
 *  Created on: Sep 29, 2020
 *      Author: X-Phase s.r.l., Leonardo Volpi
 */

#ifndef MAIN_X_JSN_H_
#define MAIN_X_JSN_H_

/* --------------------- INCLUDES ------------------------ *
 * ------------------------------------------------------- */
#include "utility.h"

/* --------------------- DEFINES ------------------------- *
 * ------------------------------------------------------- */

/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */

/* --------------------- FUNCTIONS ----------------------- *
 * ------------------------------------------------------- */
void jsn_add_key(char *j, char *key);

void jsn_set_int_key( char *j, int *pval, u32 len, u32 inc, u32 len2, u32 inc2 );

void jsn_set_float_key( char *j, float *pval, u32 len, u32 inc, u32 len2, u32 inc2 );

void jsn_set_str_key( char *j, const char *strval);

void jsn_add_obj(char *j, char *key);

void jsn_add_array(char *j, char *key);

void jsn_array_cls(char *j);

void jsn_cls(char *j);

#endif /* MAIN_X_JSN_H_ */
