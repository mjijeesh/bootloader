/*
 * common.c
 *
 *  Created on: 12-Jul-2018
 *      Author: jijeesh
 */



#include "drivers/mss_uart/mss_uart.h"  /* For getting GPIO read/write functions */
#include "CMSIS/system_m2sxxx.h"
#include "hal.h"
#include "common.h"


uint32_t int_to_hex_int(uint32_t value, uint8_t * p_result, uint32_t result_size);
uint32_t int_to_dec_int(uint32_t value, uint8_t * p_result, uint32_t result_size);

void dp_display_text(uint8_t * text)
{




	MSS_UART_polled_tx_string(&g_mss_uart1,text);


   // UART_send (&g_stdio_uart, (unsigned char *)text,length);

}

void dp_display_value(uint32_t value,uint32_t descriptive)
{
	/* User Specific Code */
	uint8_t value_text[256];

	    int text_size;
	    if (descriptive == HEX)
	    {
	        text_size = int_to_hex_int( value, value_text, 8);
	        MSS_UART_polled_tx_string(&g_mss_uart1, "0x");
	        MSS_UART_polled_tx(&g_mss_uart1, value_text, text_size);
	    }
	    else
	    {
	        text_size = int_to_dec_int( value, value_text, 8);
	        MSS_UART_polled_tx(&g_mss_uart1, value_text, text_size);
	    }

	    return;
}
void dp_display_array(uint8_t *value,uint32_t bytes, uint32_t descriptive)
{
	int idx,cnt;
	    cnt =0;
	    for (idx=0;idx<bytes;idx++)
	    {
	        dp_display_value(value[idx], descriptive);
	        dp_display_text(" ");
	        cnt++;

	        if ( cnt >= 7) {

	        	 cnt = 0;
	        	 dp_display_text("\r\n");
	        }


	    }
	    return;

}


#define NB_NIBBLES_IN_INT		8

uint32_t int_to_hex_int(uint32_t value, uint8_t * p_result, uint32_t result_size)
{
    int nibble_idx, nb_nibbles;
    uint8_t conv_array[NB_NIBBLES_IN_INT];
    uint32_t uvalue;
    nibble_idx = 0;
    uvalue = (uint32_t)value;

    do {
        int nibble = uvalue & 0x0F;

        if ( nibble < 10 )
            conv_array[nibble_idx] = nibble + '0';
        else
        conv_array[nibble_idx] = nibble  - 10 + 'A';
        uvalue = (uvalue >> 4);
        nibble_idx++;
    } while ( ( nibble_idx < NB_NIBBLES_IN_INT ) && ( uvalue > 0 ) );

    nb_nibbles = nibble_idx;
    for ( nibble_idx = 0; (nibble_idx < nb_nibbles) && (nibble_idx < result_size) ;nibble_idx++ )
    {
        p_result[nibble_idx] = conv_array[nb_nibbles - nibble_idx - 1];
    }
    return nibble_idx;
}

uint32_t int_to_dec_int(uint32_t value, uint8_t * p_result, uint32_t result_size)
{

    uint8_t conv_array[NB_NIBBLES_IN_INT];
    uint32_t uvalue;
    uint32_t remainder;
    uint32_t digit_idx,nb_digits;

    uvalue = (uint32_t)value;
    digit_idx=0;
    if (uvalue)
    {
        while (uvalue)
        {
            remainder = uvalue % 10;
            conv_array[digit_idx] = remainder + '0';
            uvalue /= 10;
            digit_idx++;
        }
    }
    else
    {
        conv_array[digit_idx] = '0';
        digit_idx++;
    }


    nb_digits = digit_idx;
    for ( digit_idx = 0; (digit_idx < nb_digits); digit_idx++ )
    {
        p_result[digit_idx] = conv_array[nb_digits - digit_idx - 1];
    }
    return digit_idx;
}


//#include "../CMSIS/a2fxxxm3.h"



int xatoi (                                             /* 0:Failed, 1:Successful */
		       uint8_t  **str,                          /* Pointer to pointer to the string */
		       uint32_t  *res                             /* Pointer to the valiable to store the value */
)
{
                unsigned long val;
                unsigned char c, r, s = 0;


                *res = 0;

                while ((c = **str) == ' ') (*str)++;               /* Skip leading spaces */

                if (c == '-') {                         /* negative? */
                                s = 1;
                                c = *(++(*str));
                }

                if (c == '0') {
                                c = *(++(*str));
                                switch (c) {
                                case 'x':                                /* hexdecimal */
                                case 'X':

                                                r = 16; c = *(++(*str));
                                                break;
                                case 'b':                                /* binary */
                                case 'B':

                                                r = 2; c = *(++(*str));
                                                break;
                                default:
                                                if (c <= ' ') return 1;          /* single zero */
                                                if (c < '0' || c > '9') return 0;          /* invalid char */
                                                r = 8;                      /* octal */
                                }
                } else {
                                if (c < '0' || c > '9') return 0;          /* EOL or invalid char */
                                r = 10;                                    /* decimal */
                }

                val = 0;
                while (c > ' ') {
                                if (c >= 'a') c -= 0x20;
                                c -= '0';
                                if (c >= 17) {
                                                c -= 7;
                                                if (c <= 9) return 0;           /* invalid char */
                                }
                                if (c >= r) return 0;                           /* invalid char for current radix */
                                val = val * r + c;
                                c = *(++(*str));
                }
                if (s) val = 0 - val;                                               /* apply sign if needed */

                *res = val;
                return 1;
}



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Convert an Integer to a string
  * @param  str: The string
  * @param  intnum: The intger to be converted
  * @retval None
  */
void Int2Str(uint8_t* str, int32_t intnum)
{
  uint32_t i, Div = 1000000000, j = 0, Status = 0;

  for (i = 0; i < 10; i++)
  {
    str[j++] = (intnum / Div) + 48;

    intnum = intnum % Div;
    Div /= 10;
    if ((str[j-1] == '0') & (Status == 0))
    {
      j = 0;
    }
    else
    {
      Status++;
    }
  }
}






uint8_t  getParity(uint32_t n)
{
	uint8_t parity = 0;
    while (n)
    {
        parity = !parity;
        n      = n & (n - 1);
    }
    return parity;
}
