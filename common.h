/*
 * common.h
 *
 *  Created on: 12-Jul-2018
 *      Author: jijeesh
 */

#ifndef COMMON_H_
#define COMMON_H_



#define HEX 0u
#define DEC 1u
#define CHR 2u


void dp_display_value(uint32_t value,uint32_t descriptive);
void dp_display_array(uint8_t *value,uint32_t bytes, uint32_t descriptive);
void Int2Str(uint8_t* str, int32_t intnum);

#endif /* COMMON_H_ */
