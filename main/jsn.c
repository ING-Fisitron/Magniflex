/*
 * x-jsn.c
 *
 *  Created on: Sep 29, 2020
 *      Author: X-Phase s.r.l., Leonardo Volpi
 */


/* --------------------- INCLUDES ------------------------ *
 * ------------------------------------------------------- */
#include "jsn.h"

/* --------------------- DEFINES ------------------------- *
 * ------------------------------------------------------- */

/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */
static const char *TAG = "XJSN";



/* --------------------- FUNCTIONS ----------------------- *
 * ------------------------------------------------------- */
void jsn_add_key(char *j, char *key) {
	char tmpj[500];
	if (j[0] != '{') {
		j[0] = '{';
		j[1] = 0;
	}
	else if ( j[strlen(j)-1] != '{') {
		strcat(j,",");
	}
	sprintf(tmpj, "\"%s\"", key);
	strcat(j,tmpj);
}

void jsn_set_int_key( char *j, int *pval, u32 len, u32 inc, u32 len2, u32 inc2 ) {
	char tmpj[500];
	strcat(j,":");
	if ( len2 > 1 ) { // Add item value as array.

		strcat(j,"[");
		for (int a = 0; a < len2; a+=inc2) {

			strcat(j,"[");
			for (int k = a*len; k < (len+a*len); k+=inc) {
				sprintf(tmpj,"%d", pval[k]);
				strcat(tmpj,",");
				strcat(j,tmpj);
			}
			j[strlen(j)-1] = ']';
			strcat(j,",");

		}
		j[strlen(j)-1] = ']';

	}
	else if ( len > 1 ) { // Add item value as array.
		strcat(j,"[");
		for (int k = 0; k < len; k+=inc) {
			sprintf(tmpj,"%d", pval[k]);
			strcat(tmpj,",");
			strcat(j,tmpj);
		}
		j[strlen(j)-1] = ']';
	} else {
		sprintf(tmpj,"%d",pval[0]);
		strcat(j,tmpj);
	}
}

void jsn_set_float_key( char *j, float *pval, u32 len, u32 inc, u32 len2, u32 inc2 ) {
	char tmpj[500];
	strcat(j,":");
	if ( len2 > 1 ) { // Add item value as array.

		strcat(j,"[");
		for (int a = 0; a < len2; a+=inc2) {

			strcat(j,"[");
			for (int k = a*len; k < (len+a*len); k+=inc) {
				sprintf(tmpj,"%.3f", pval[k]);
				strcat(tmpj,",");
				strcat(j,tmpj);
			}
			j[strlen(j)-1] = ']';
			strcat(j,",");

		}
		j[strlen(j)-1] = ']';

	}
	else if ( len > 1 ) { // Add item value as array.
		strcat(j,"[");
		for (int k = 0; k < len; k+=inc) {
			sprintf(tmpj,"%.3f", pval[k]);
			strcat(tmpj,",");
			strcat(j,tmpj);
		}
		j[strlen(j)-1] = ']';
	} else {
		sprintf(tmpj,"%.3f",pval[0]);
		strcat(j,tmpj);
	}
}

void jsn_set_str_key( char *j, const char *strval) {
	char tmpj[500];
	sprintf(tmpj,":\"%s\"",strval);
	strcat(j,tmpj);
}

void jsn_add_obj(char *j, char *key) {
	char tmpj[500];
	if ( j[strlen(j)-1] != '{' && j[strlen(j)-1] != '[' ) {
		strcat(j,",");
	}
	if ( strlen(key) == 0 ) {
		strcat(j,"{");
	} else {
		sprintf(tmpj, "\"%s\":{", key);
		strcat(j,tmpj);
	}
}

void jsn_add_array(char *j, char *key) {
	char tmpj[500];
	if ( j[strlen(j)-1] != '{' && j[strlen(j)-1] != '[' ) {
		strcat(j,",");
	}
	if ( strlen(key) == 0 ) {
		strcat(j,"[");
	} else {
		sprintf(tmpj, "\"%s\":[", key);
		strcat(j,tmpj);
	}
}

void jsn_array_cls(char *j) {
	strcat(j,"]");
}

void jsn_cls(char *j) {
	strcat(j,"}");
}
