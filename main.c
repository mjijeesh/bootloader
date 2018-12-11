/*******************************************************************************
 * (c) Copyright 2016-2017 Microsemi SoC Products Group. All rights reserved.
 *
 *  Simple boot-loader example program.
 *  This sample project is targeted at a RISC-V design running on the M2S150
 *  development board.
 *  You can program the SPI Flash from a command line program and have the
 *  boot-loader load a program from SPI Flash and jump to it.
 *  These actions are driven from a serial command line interface.
 *
 * SVN $Revision: $
 * SVN $Date: $
 */
#include <unistd.h>
#include <string.h>


#include "hal/hal.h"

#include "drivers/mss_uart/mss_uart.h"
#include "drivers/mss_gpio/mss_gpio.h"
#include "drivers/mss_timer/mss_timer.h"
#include "drivers/w25q64fvssig/w25q64fvssig.h"
#include "drivers/spi_flash/spi_flash.h"
#include "drivers/mss_pdma/mss_pdma.h"
#include "CMSIS/system_m2sxxx.h"




#include "ymodem/ymodem.h"
#include "common.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t file_name[FILE_NAME_LENGTH];
extern uint8_t file_size[FILE_SIZE_LENGTH + 1];
//extern int32_t file_Size;

/* Manufacture and device IDs for Winbond Electronics W25Q64FVSSIG SPI Flash. */
#define FLASH_MANUFACTURER_ID   (uint8_t)0xEF
#define FLASH_DEVICE_ID         (uint8_t)0x16

#define BUFFER_A_SIZE   		3000
#define MAX_IMAGE_FILE_SIZE     1024 * 1024 * 8
#define SPI_FLASH_COPY_LOCATION   0x0000


/*
 * Static global variables
 */
static uint8_t g_flash_wr_buf[BUFFER_A_SIZE];
static uint8_t g_flash_rd_buf[BUFFER_A_SIZE];




#define FLASH_SECTOR_SIZE   65536 // Sectors are 64K bytes
#define FLASH_SECTORS       128   // This is an 8MB part with 128 sectors of 64KB
#define FLASH_BLOCK_SIZE    4096  // We will use the 4K blocks for this example
#define FLASH_SEGMENT_SIZE  256   // Write segment size

#define FLASH_BLOCK_SEGMENTS ( FLASH_BLOCK_SIZE / FLASH_SEGMENT_SIZE )

#define FLASH_BYTE_SIZE		(FLASH_SECTOR_SIZE * FLASH_SECTORS)
#define LAST_BLOCK_ADDR     (FLASH_BYTE_SIZE - FLASH_BLOCK_SIZE)


static void Init_GPIO(void);
static void Init_UART(void);
static void Init_Timer(void);



static uint32_t Serial_Download(uint8_t *dest_address);
static void write_program_to_flash( uint8_t * src_addr, uint32_t file_size);
static void read_program_from_flash(uint8_t *read_buf, uint32_t read_byte_length);
static void read__flash(uint8_t *read_buf,uint32_t read_byte_size);

//static uint32_t Serial_Download(uint8_t *dest_address);

static void DDR_test (uint32_t length);
static void FLASH_test (uint8_t *src_addr);
static uint8_t verify_write(uint8_t* write_buff, uint8_t* read_buff, uint16_t size);

static void mem_test(uint8_t *address);


static uint32_t rx_app_file(uint8_t *dest_address);

static void Bootloader_JumpToApplication(uint32_t stack_location, uint32_t reset_vector);

//static int read_program_from_flash(uint8_t *read_buf, uint32_t read_byte_length);

//static int write_program_to_flash(uint8_t *write_buf, uint32_t file_size);

static void show_progress(void);
static void display_options(void);



/*
 * Data structure stored at the beginning of SPI flash to indicate the suize of
 * data stored inside the SPI flash. This is to avoid having to read the entire
 * flash content at boot time.
 * This data structure is one flash segment long (256 bytes).
 */
typedef struct
{
    uint32_t validity_key;
    uint32_t spi_content_byte_size;
    uint8_t spi_content_file_name[FILE_NAME_LENGTH];
    uint8_t spi_content_file_size[FILE_SIZE_LENGTH+1];
    uint32_t dummy[32];
} flash_content_t;

/*
 * Base address of DDR memory where executable will be loaded.
 */
#define DDR_BASE_ADDRESS    0xA0000000

/*
 * Delay loop down counter load value.
 */
#define DELAY_LOAD_VALUE     0x00008000

/*
 * Bit mask identifying the DIP switch used to indicate whether the boot loader
 * should load and launch the application on system reset or stay running to
 * allow a new image to be programming into the SPI flash.
 */
#define BOOTLOADER_DIP_SWITCH   0x00000080

/*
 * Key value used to determine if valid data is contained in the SPI flash.
 */
#define SPI_FLASH_VALID_CONTENT_KEY     0xB5006BB1

/*
 * CoreGPIO instance data.
 */
//gpio_instance_t g_gpio;
uint32_t timer1_load_value;
uint32_t gpio_pattern,g_gpio_pattern;

volatile uint32_t g_10ms_count;
volatile uint32_t g_state;


/******************************************************************************
 * Maximum receiver buffer size.
 *****************************************************************************/
#define MAX_RX_DATA_SIZE    256

/******************************************************************************
 * CoreUARTapb instance data.
 *****************************************************************************/
//UART_instance_t g_uart;

/******************************************************************************
 * Instruction message. This message will be transmitted over the UART to
 * HyperTerminal when the program starts.
 *****************************************************************************/
const uint8_t g_greeting_msg[] =
"\r\n\r\n\
===============================================================================\r\n\
                    Microsemi RISC-V Boot Loader v0.2.2\r\n\
===============================================================================\r\n\
 This boot loader provides the following features:\r\n\
    - Load a program into DDR memory using the YModem file transfer protocol.\r\n\
    - Write a program into the board's SPI flash. The executable must first be\r\n\
      loaded into the board's DDR memory using the YModem file transfer\r\n\
      protocol.\r\n\
    - Load a program from SPI flash into external DDR memory and launch the\r\n\
      program.\r\n\
    - Automatically load and execute a program from SPI flash depending on DIP\r\n\
      switch 0 position.\r\n\
";

const uint8_t g_instructions_msg[] =
"\r\n\r\n\
-------------------------------------------------------------------------------\r\n\
 Type 0 to show this menu\n\r\
 Type 1 to start Ymodem transfer to DDR memory\n\r\
 Type 2 to copy program from DDR to flash \n\r\
 Type 3 to copy program from flash to DDR \n\r\
 Type 4 to start program loaded in DDR \n\r\
 Type 5 to test Flash device 0\n\r\
 Type 6 to test DDR\n\r\
";


/*==============================================================================
  Messages displayed over the UART.
 */
const uint8_t g_option_msg[] =
" Enter desired action to perform in the below mentioned sequence: \r\n\
   0. Display This Menu \r\n\
   1. Downlaod file to DDR ( YMODEM) \r\n\
   2. Copy File from DDR to SPI Flash \r\n\
   3. Copy File from SPI Flash to DDR \r\n\
   4. Start Program execution from DDR \r\n\
   5. Test SPI Flash memory \r\n\
   6. Test DDR memory \r\n\
 Enter valid selection [1-6]: \r\n";

const uint8_t g_separator[] =
"\r\n----------------------------------------------------------------------\r\n";


const uint8_t g_boot_dip_switch_off_msg[] =
"\r\n\
-------------------------------------------------------------------------------\r\n\
 Boot loader jumper/switch set to start application.\r\n\
 Toggle DIP switch 0 and reset the board if you wish to execute the boot loader\r\n\
 to load another application into the SPI flash.\r\n\
";

const uint8_t g_boot_dip_switch_on_msg[] =
"\r\n\
-------------------------------------------------------------------------------\r\n\
 Boot loader jumper/switch set to stay in boot loader.\r\n\
 Toggle DIP switch 0 and reset the board if you wish to automatically execute\r\n\
 the application stored in the SPI flash on reset.\r\n\
";

const uint8_t g_load_executable_msg[] =
"\r\n\
-------------------------------------------------------------------------------\r\n\
 Loading application from SPI flash into DDR memory.\r\n";

const uint8_t g_run_executable_msg[] =
"\r\n\
-------------------------------------------------------------------------------\r\n\
 Executing application in DDR memory.\r\n\
-------------------------------------------------------------------------------\r\n\
 \r\n";

/******************************************************************************
 * Timer load value. This value is calculated to result in the timer timing
 * out after after 1 second with a system clock of 24MHz and the timer
 * prescaler set to divide by 1024.
 *****************************************************************************/
#define TIMER_LOAD_VALUE    23437

/******************************************************************************
 * GPIO instance data.
 *****************************************************************************/
//gpio_instance_t g_gpio;




size_t
UART_Polled_Rx
(
    mss_uart_instance_t * this_uart,
    uint8_t * rx_buff,
    size_t buff_size
)
{
    size_t rx_size = 0U;

    while( rx_size < buff_size )
    {
       while ( ((this_uart->hw_reg->LSR) & 0x1) != 0U  )
       {
           rx_buff[rx_size] = this_uart->hw_reg->RBR;
           ++rx_size;
       }
    }

    return rx_size;
}




/*-------------------------------------------------------------------------*//**
 * main() function.
 */
int main()
{
	uint8_t rx_data[MAX_RX_DATA_SIZE];
	size_t rx_size;
	char wait_in_bl;


    g_10ms_count = 0;

    SystemCoreClockUpdate();

    /* Turn off the watchdog */
        SYSREG->WDOG_CR = 0;



      //  Init_Timer();




    /**************************************************************************
     * Initialize the RISC-V platform level interrupt controller.
     *************************************************************************/
   // PLIC_init();

    /**************************************************************************
     * Initialize the CoreGPIO driver with the base address of the CoreGPIO
     * instance to use and the initial state of the outputs.
     *************************************************************************/
        Init_GPIO();

    /**************************************************************************
      * Initialize CoreUARTapb with its base address, baud value, and line
      * configuration.
      *************************************************************************/
        Init_UART();
	/**************************************************************************
		  * Initialize The MSS_SPI_0 and read Device Information from Flash Memory.
    *************************************************************************/

        FLASH_init();



    /**************************************************************************
     * Display greeting message message.
     *************************************************************************/
	MSS_UART_polled_tx_string( &g_mss_uart1, g_greeting_msg);

    /**************************************************************************
     * Set up the system tick timer
     *************************************************************************/
    SysTick_Config(SystemCoreClock / 100);

   	/*
	 * Check to see if boot-loader switch is set
	 */



    /* new 2 lines by JM */

    wait_in_bl = 1;
    MSS_UART_polled_tx_string( &g_mss_uart1, g_instructions_msg);

    while(wait_in_bl == 1)
    {
    	static uint32_t image_file_size = 0;
    	static uint32_t readback_size = (126 * 1024) /*FLASH_BYTE_SIZE*/;

         /**********************************************************************
         * Read data received by the UART.
         *********************************************************************/
        //rx_size = UART_Polled_Rx( &g_mss_uart1,rx_data, sizeof(rx_data) );
    	rx_size = UART_Polled_Rx( &g_mss_uart1,rx_data, 1) ;

        /**********************************************************************
         * Echo back data received, if any.
         *********************************************************************/
        if ( rx_size > 0 )
        {
        	MSS_UART_polled_tx(&g_mss_uart1, rx_data, rx_size );

            switch(rx_data[0])
            {
                case '0':
                	//MSS_UART_polled_tx_string( &g_mss_uart1, g_instructions_msg);
                	display_options();
                    break;
                case '1':
                	//file_size = rx_app_file((uint8_t *)DDR_BASE_ADDRESS);

                	image_file_size =  Serial_Download((uint8_t *)DDR_BASE_ADDRESS);
                    break;
                case '2':
                    write_program_to_flash((uint8_t *)DDR_BASE_ADDRESS, image_file_size);
                    readback_size = image_file_size;
                    break;
                case '3':
                   //read_program_from_flash((uint8_t *)DDR_BASE_ADDRESS, readback_size);
                   read__flash((uint8_t *)DDR_BASE_ADDRESS,readback_size );
                   display_options();
                    break;
                case '4':
                	/* Populate with stack and reset vector address i.e. The first two words of the program */
                    Bootloader_JumpToApplication((uint32_t)0xA0000000, (uint32_t)0xA0000004);
                    break;
                case '5':
                    /* Works- but needed to blow open Libero "RTG4 DDR Memory controller with initialization"
                     * and edit with parameters taken from design on web site- see ABC program below */
                    // test_flash();
                    // test_flash_w25();
                 	// test_flash((uint8_t *)DDR_BASE_ADDRESS,13000);
                	 //DDR_test (2048);
                	FLASH_test((uint8_t *)DDR_BASE_ADDRESS);

                    break;
                case '6':
                    /*
                     * Sanity check on DDR working
                     */



                   // mem_test((uint8_t *)DDR_BASE_ADDRESS);
                    DDR_test (2048);
                    break;
                case '7':

                	//test_flash((uint8_t *)DDR_BASE_ADDRESS,13000);
					/* test soft reset- with CoreBootStrap */
                	/* wipe reset vector */
                	/* choose this option */
                	/* system should reboot correctly */
//<CJ>                	__disable_irq();
//<CJ>                	*acr = 0x04;  						/*; make sure ITCM mapped to 0x10000000 only */
//<CJ>                	asm ("DSB  ");                      /*; ensure that store completes before      */
                	                                    /*; executing the next instruction  		*/
//<CJ>                	asm ("ISB  ");                      /*; executing synchronization instruction   */
                	/*Set reset vector in LSRAM tyo zero */
//<CJ>                	*(uint32_t *)0x00000004 = 0;
//<CJ>                	NVIC_SystemReset();		/* RESET the system- program will be loaded by CoreBootStrap */
			break;
            }
        }
    }
    MSS_UART_polled_tx_string( &g_mss_uart1, g_load_executable_msg);
   // read_program_from_flash((uint8_t *)DDR_BASE_ADDRESS, (128 * 1024));
   // MSS_UART_polled_tx_string( &g_mss_uart1, g_run_executable_msg);
   // Bootloader_JumpToApplication(0x70000000, 0x70000004);
    /* will never reach here! */
    while(1);
    return 0;
}


/*------------------------------------------------------------------------------
  Display user option to execute IAP service.
 */
static void display_options(void)
{
    uint8_t rx_size = 0;
    uint8_t rx_data = 0;

    MSS_UART_polled_tx_string(&g_mss_uart1, (const uint8_t *)"\r\nPress any key to continue...");
    do {
        rx_size = MSS_UART_get_rx(&g_mss_uart1, &rx_data,1);
    } while(rx_size == 0);

    MSS_UART_polled_tx_string(&g_mss_uart1, g_separator);
    MSS_UART_polled_tx (&g_mss_uart1, g_option_msg,sizeof(g_option_msg));
    return;
}


/*
 *  Test flash on RTG4
 */

static void DDR_test (uint32_t length) {


		uint8_t img_buffer[1024];
		uint32_t size;
		uint32_t srcAddr=0;
		unsigned int ii=0, jj =0;

		unsigned char *destAddr = (unsigned char *)0xA0000000;

			for(ii=0; ii<length; ii++)
			{
				*destAddr++ = 0xAA;  //img_buffer[ii];
			}

			MSS_UART_polled_tx_string( &g_mss_uart1, "  DDR Write Test Success \n\r" );


		}



/*
 * Simple sanity check
 */
static void mem_test(uint8_t *address)
{
    volatile uint8_t value=2;
    volatile uint32_t value32=3;
    *address = 1;
    value = *address;
    value32 = (uint32_t)*address;
    if((value32 == value) &&(value == 1))
    	MSS_UART_polled_tx_string( &g_mss_uart1, "  Read/Write success\n\r" );
    else
    	MSS_UART_polled_tx_string( &g_mss_uart1, "  Read/Write fail\n\r" );
}

static void FLASH_test (uint8_t *src_addr){

	 uint8_t manufacturer_id;
     uint8_t device_id;
     volatile uint32_t errors = 0;
	 uint32_t address = 0;
	 uint16_t loop_count;
	 uint8_t *addr_ptr;

	 addr_ptr = src_addr;

    FLASH_init();

    /*--------------------------------------------------------------------------
    		 * Check SPI Flash part manufacturer and device ID.
    */
	FLASH_read_device_id(&manufacturer_id, &device_id);
	if ((manufacturer_id == FLASH_MANUFACTURER_ID) || (device_id == FLASH_DEVICE_ID))
	{
		MSS_UART_polled_tx_string( &g_mss_uart1,"Winbond SPI Flash Detectd");


	} else

	{
		++errors;
		MSS_UART_polled_tx_string( &g_mss_uart1,"Un Supported Flash Memory Detected");


	}

    MSS_UART_polled_tx_string(&g_mss_uart1,"\n\n Flash Initialised\n\r");


    /*--------------------------------------------------------------------------
         * Initialize the write and read Buffers
        */
        for(loop_count = 0; loop_count < (BUFFER_A_SIZE/2); loop_count++)
        {
            g_flash_wr_buf[loop_count] = 0x44 + loop_count;
            g_flash_rd_buf[loop_count] = 0x00;

            //addr_ptr[loop_count] = 0xAA + loop_count;
            *addr_ptr++ = 0x00+ loop_count;
        }
        for(loop_count = (BUFFER_A_SIZE/2); loop_count < BUFFER_A_SIZE; loop_count++)
        {
            g_flash_wr_buf[loop_count] = 0x33;
            g_flash_rd_buf[loop_count] = 0x00;
        }



    FLASH_global_unprotect();
    MSS_UART_polled_tx_string(&g_mss_uart1,"\n\n Flash Write Enabled\n\r");


    FLASH_erase_4k_block(0);

    MSS_UART_polled_tx_string(&g_mss_uart1,"\n\n Flash Erase Completed\n\r");



    /*--------------------------------------------------------------------------
     * Write Data to Flash.
    */
    address = 200;
    //FLASH_program(address, g_flash_wr_buf, sizeof(g_flash_wr_buf));

    //FLASH_program(address, addr_ptr,256);
    FLASH_program(address, src_addr,256);  // addr_ptr is incremented so can't use
    //dp_display_array(g_flash_wr_buf,256, HEX);

    /*--------------------------------------------------------------------------
     * Read Data From Flash.
    */
    address = 200;
    //FLASH_read(address, g_flash_rd_buf, sizeof(g_flash_wr_buf));
    FLASH_read(address, g_flash_rd_buf, 256);

    dp_display_array(g_flash_rd_buf,256, HEX);



    //errors = verify_write(g_flash_rd_buf, g_flash_wr_buf, sizeof(g_flash_wr_buf));



    MSS_UART_polled_tx_string(&g_mss_uart1,"\n\n Number of Error Encountered: \n\r");


    dp_display_value(errors,DEC);


}

/***************************************************************************//**
 * Read the date from SPI FLASH and compare the same with write buffer.
 */
static uint8_t verify_write(uint8_t* write_buff, uint8_t* read_buff, uint16_t size)
{
    uint8_t error = 0;
    uint16_t index = 0;

    while(size != 0)
    {
        if(write_buff[index] != read_buff[index])
        {
            error = 1;
            break;
        }
        index++;
        size--;
    }

    return error;
}





/*
 * Put image received via ymodem into memory
 */
static uint32_t rx_app_file(uint8_t *dest_address)
{
	uint32_t received;
    uint8_t *g_bin_base = (uint8_t *)dest_address;
    uint32_t g_rx_size = 1024 * 1024 * 8;

    MSS_UART_polled_tx_string( &g_mss_uart1, "\r\n------------------------ Starting YModem file transfer ------------------------\r\n" );
    MSS_UART_polled_tx_string( &g_mss_uart1, "Please select file and initiate transfer on host computer.\r\n" );

    received = ymodem_receive(g_bin_base, g_rx_size);

    MSS_UART_polled_tx_string( &g_mss_uart1, "\nFile Transfer Completed.\r\n" );

    return received;
}



/*
 * Put image received via ymodem into memory
 */
/*
 *
 */
static void show_progress(void)
{
    static uint32_t progress_count = 0;
    static uint32_t dot_count = 0;

    if (progress_count > 10)
    {
    	MSS_UART_polled_tx_string( &g_mss_uart1, (const uint8_t*)"." );
        progress_count = 0;
        ++dot_count;

    }
    ++progress_count;

    if (dot_count > 78)
    {
    	MSS_UART_polled_tx_string( &g_mss_uart1, (const uint8_t*)"\r\n" );
        dot_count = 0;
    }
}

/*------------------------------------------------------------------------------
 * Count the number of elapsed milliseconds (SysTick_Handler is called every
 * 10mS so the resolution will be 10ms). Rolls over every 49 days or so...
 *
 * Should be safe to read g_10ms_count from elsewhere.
 */
static uint32_t Serial_Download(uint8_t *dest_address)
{
	uint32_t received_size;
    uint8_t *g_bin_base = (uint8_t *)dest_address;
    //uint32_t g_rx_size = 1024 * 1024 * 8;
   // uint8_t Number[10] = "          ";
   // int32_t Size = 0;



    MSS_UART_polled_tx_string( &g_mss_uart1, "Waiting for the file to be sent ... (press 'a' to abort)\n\r");
    MSS_UART_polled_tx_string( &g_mss_uart1, "Please select file and initiate transfer on host computer.\r\n" );

    received_size = ymodem_receive(g_bin_base, MAX_IMAGE_FILE_SIZE);
   // Size = str_to_u32(file_size);

    if (received_size > 0)
      {
    	MSS_UART_polled_tx_string(&g_mss_uart1, "\n\n\r File Download  Completed Successfully!\n\r------\r\n Name: ");
    	MSS_UART_polled_tx_string(&g_mss_uart1,file_name);

    	MSS_UART_polled_tx_string(&g_mss_uart1,"\n\r Size: ");
    	//dp_display_value((uint32_t)file_Size, DEC);
    	MSS_UART_polled_tx_string(&g_mss_uart1,file_size);
        MSS_UART_polled_tx_string(&g_mss_uart1," Bytes\r\n");

        MSS_UART_polled_tx_string(&g_mss_uart1,"----------------------\n\r");
      }


    	  else if (received_size == -1)
    	    {
    	  	  MSS_UART_polled_tx_string(&g_mss_uart1,"\n\n\rThe image size is higher than the allowed space memory!\n\r");
    	    }
    	    else if (received_size == -2)
    	    {
    	  	  MSS_UART_polled_tx_string(&g_mss_uart1,"\n\n\rVerification failed!\n\r");
    	    }
    	    else if (received_size== -3)
    	    {
    	  	  MSS_UART_polled_tx_string(&g_mss_uart1,"\r\n\nAborted by user.\n\r");
    	    }
    	    else
    	    {
    	  	  MSS_UART_polled_tx_string(&g_mss_uart1,"\n\rFailed to receive the file!\n\r");
    	    }

    return received_size;
}

static void write_program_to_flash( uint8_t * src_addr, uint32_t image_file_size)

{
	uint32_t byte_Read = 0;
	uint32_t remaining_size = 0;
	uint32_t flash_address = SPI_FLASH_COPY_LOCATION;
	uint32_t bitstream_spi_addr = SPI_FLASH_COPY_LOCATION;
	int8_t tx_complete;


	 uint8_t manufacturer_id;
	 uint8_t device_id;
	 volatile uint32_t errors = 0;
	 uint32_t address = 0;
	 uint32_t count;
	 uint8_t * addr_ptr;
	 uint8_t * write_buf_ptr;

	 write_buf_ptr = src_addr;

	 addr_ptr = src_addr;

	 FLASH_init();

	    /*--------------------------------------------------------------------------
	    		 * Check SPI Flash part manufacturer and device ID.
	    */
		FLASH_read_device_id(&manufacturer_id, &device_id);

		if ((manufacturer_id == FLASH_MANUFACTURER_ID) || (device_id == FLASH_DEVICE_ID))
		{
			MSS_UART_polled_tx_string( &g_mss_uart1,"Winbond SPI Flash Detectd");


		} else

		{
			++errors;
			MSS_UART_polled_tx_string( &g_mss_uart1,"Un Supported Flash Memory Detected");


		}

	    MSS_UART_polled_tx_string(&g_mss_uart1,"\n\n Flash Initialised\n\r");



	    FLASH_global_unprotect();



	MSS_UART_polled_tx_string(&g_mss_uart1, (const uint8_t*)"\r\n Copying the selected file from DDR  into SPI FLASH.\r\n Please wait..");

	MSS_UART_polled_tx_string(&g_mss_uart1, (const uint8_t*)"\r\n  File Size   : ");

				            dp_display_value((uint32_t)image_file_size, DEC);

			uint32_t nb_blocks_to_write;
			nb_blocks_to_write = (image_file_size / FLASH_SEGMENT_SIZE);
			    if ((image_file_size % FLASH_SEGMENT_SIZE) > 0)
			    {
			        ++nb_blocks_to_write;
			    }

			    MSS_UART_polled_tx_string(&g_mss_uart1, (const uint8_t*)"\r\n  Number of Segments to Erase : ");

			    dp_display_value((uint32_t)nb_blocks_to_write, DEC);

			    for( count = 0; count != nb_blocks_to_write; ++count )
			    {

			        /*----------------------------------------------------------------------
			         * at the start of each sector we need to make sure it is unprotected
			         * so we can erase blocks within it. The spi_flash_write() function
			         * unprotects the sector as well but we need to start erasing before the
			         * first write takes place.
			         */
			        /*----------------------------------------------------------------------
			         * At the start of each 4K block we issue an erase so that we are then
			         * free to write anything we want to the block. If we don't do this the
			         * write may fail as we can only effectively turn 1s to 0s when we
			         * write. For example if we have an erased location with 0xFF in it and
			         * we write 0xAA to it first and then later on write 0x55, the resulting
			         * value is 0x00...
			         */
			        if(0 == (flash_address % FLASH_BLOCK_SIZE))
			        {
			        	 //result = spi_flash_control_hw( SPI_FLASH_SECTOR_UNPROTECT, flash_address, NULL );
			            //result = spi_flash_control_hw( SPI_FLASH_4KBLOCK_ERASE, flash_address , NULL );
			            FLASH_erase_4k_block(flash_address);

			            MSS_UART_polled_tx_string(&g_mss_uart1, (const uint8_t*)"\r\n  Erasing Segment Addr  : ");

			            dp_display_value((uint32_t)flash_address, HEX);
			        }
			        /*----------------------------------------------------------------------
			         * Write our values to the FLASH, read them back and compare.
			         * Placing a breakpoint on the while statement below will allow
			         * you break on any failures.
			         */

			        //spi_flash_write( flash_address, write_buf_ptr, FLASH_SEGMENT_SIZE );
			        FLASH_program(flash_address, write_buf_ptr, FLASH_SEGMENT_SIZE);  // addr_ptr is incremented so can't use

			        MSS_UART_polled_tx_string(&g_mss_uart1, (const uint8_t*)"\r\n  Programming Segment   : ");

			        dp_display_value((uint32_t)flash_address, HEX);
			        MSS_UART_polled_tx_string(&g_mss_uart1, (const uint8_t*)" Completed ");



			       // spi_flash_read ( flash_address, read_buffer, FLASH_SEGMENT_SIZE );
			        FLASH_read(flash_address, g_flash_rd_buf, FLASH_SEGMENT_SIZE);

			        errors = verify_write(g_flash_rd_buf, write_buf_ptr, FLASH_SEGMENT_SIZE);
		            MSS_UART_polled_tx_string(&g_mss_uart1,"\n\n Number of Error Encountered: \n\r");
		            dp_display_value(errors,DEC);

			        write_buf_ptr += FLASH_SEGMENT_SIZE;
			        flash_address += FLASH_SEGMENT_SIZE; /* Step to the next 256 byte chunk */
			       // show_progress();
			    }


			    /*--------------------------------------------------------------------------
			         * Record the size written in the first SPI flash segment.
			         */
			        {
			            flash_content_t flash_content;

			            flash_content.validity_key = SPI_FLASH_VALID_CONTENT_KEY;
			            flash_content.spi_content_byte_size = image_file_size;
			           // flash_content.spi_content_file_name = &file_name;
			            strcpy(flash_content.spi_content_file_name, file_name);
			            strcpy(flash_content.spi_content_file_size, file_size);


			            flash_address = LAST_BLOCK_ADDR;

			            /*----------------------------------------------------------------------
			             * at the start of each sector we need to make sure it is unprotected
			             * so we can erase blocks within it. The spi_flash_write() function
			             * unprotects the sector as well but we need to start erasing before the
			             * first write takes place.
			             */

			            if(0 == (flash_address % FLASH_BLOCK_SIZE))

			            {

			            	FLASH_erase_4k_block(flash_address);

			    			MSS_UART_polled_tx_string(&g_mss_uart1, (const uint8_t*)"\r\n  Erasing Segment Addr  : ");

			    			dp_display_value((uint32_t)flash_address, HEX);

			               // result = spi_flash_control_hw( SPI_FLASH_4KBLOCK_ERASE, flash_address , NULL );
			            }
			            /*----------------------------------------------------------------------
			             * Write our values to the FLASH, read them back and compare.
			             * Placing a breakpoint on the while statement below will allow
			             * you break on any failures.
			             */
			            //spi_flash_write( flash_address, (uint8_t *)(&flash_content), FLASH_SEGMENT_SIZE );
			            FLASH_program(flash_address, (uint8_t *)(&flash_content), FLASH_SEGMENT_SIZE);  // addr_ptr is incremented so can't use


			            //spi_flash_read ( flash_address, read_buffer, FLASH_SEGMENT_SIZE );
			            // spi_flash_read ( flash_address, read_buffer, FLASH_SEGMENT_SIZE );
						FLASH_read(flash_address, g_flash_rd_buf, FLASH_SEGMENT_SIZE);

						errors = verify_write(g_flash_rd_buf, (uint8_t *)&flash_content, FLASH_SEGMENT_SIZE);
						MSS_UART_polled_tx_string(&g_mss_uart1,"\r\n Number of Error Encountered: \n\r");
						dp_display_value(errors,DEC);

			            /*
						if( memcmp( (uint8_t *)&flash_content, read_buffer, FLASH_SEGMENT_SIZE ) )
			            {
			                while(1) // Breakpoint here will trap write faults
			                {

			                }

			            }
			           */


			        }





			    MSS_UART_polled_tx_string(&g_mss_uart1, (const uint8_t*)"\r\n Finished SPI  Programming \r\n");

			    display_options();

}

/**
 *  Read from flash
 */
static void read_program_from_flash(uint8_t *read_buf, uint32_t read_byte_length)
{
    uint16_t status;
    int flash_address = 0;

    uint32_t nb_segments_to_read;


    flash_content_t flash_content;
    uint8_t manufacturer_id;
   	 uint8_t device_id;
   	// volatile uint32_t errors = 0;
   	// uint32_t address = 0;
   	 uint32_t count,file_read_byte_length;
   	 //uint8_t * addr_ptr;
   	 uint8_t * write_buf_ptr;

    /*--------------------------------------------------------------------------
         * Flash Driver Initialization
        */
        FLASH_init();

        FLASH_global_unprotect();

       // FLASH_erase_4k_block(0);

        /*--------------------------------------------------------------------------
         * Check SPI Flash part manufacturer and device ID.
        */
        FLASH_read_device_id(&manufacturer_id, &device_id);


    /*--------------------------------------------------------------------------
     * Retrieve the size of the data previously written to SPI flash.
     */
    //spi_flash_read ( LAST_BLOCK_ADDR, (uint8_t *)&flash_content, FLASH_SEGMENT_SIZE );
    FLASH_read(LAST_BLOCK_ADDR, (uint8_t *)&flash_content, FLASH_SEGMENT_SIZE);

    if(SPI_FLASH_VALID_CONTENT_KEY == flash_content.validity_key)
    {
        file_read_byte_length = flash_content.spi_content_byte_size;
    }
    else
    {
        file_read_byte_length = 0;
    }




    MSS_UART_polled_tx_string(&g_mss_uart1,"\r\n Image File in SPI Flash : ");
    MSS_UART_polled_tx_string(&g_mss_uart1,flash_content.spi_content_file_name);
    MSS_UART_polled_tx_string(&g_mss_uart1,"\r\n Image File Size : ");
    MSS_UART_polled_tx_string(&g_mss_uart1,flash_content.spi_content_file_size);



    /*--------------------------------------------------------------------------
     * Read from flash 256 bytes increments (FLASH_SEGMENT_SIZE).
     */
    nb_segments_to_read = file_read_byte_length / FLASH_SEGMENT_SIZE;
    if((file_read_byte_length % FLASH_SEGMENT_SIZE) > 0)
    {
    	++nb_segments_to_read;
    }

    for( count = 0; count != nb_segments_to_read/2; ++count )
    {
        /*----------------------------------------------------------------------
         * Write our values to the FLASH, read them back and compare.
         * Placing a breakpoint on the while statement below will allow
         * you break on any failures.
         */

        //spi_flash_read ( flash_address, read_buf, FLASH_SEGMENT_SIZE );
        FLASH_read(flash_address, read_buf, FLASH_SEGMENT_SIZE);
        read_buf += FLASH_SEGMENT_SIZE;

        flash_address += FLASH_SEGMENT_SIZE; /* Step to the next 256 byte chunk */

       // show_progress();
    }

    MSS_UART_polled_tx_string( &g_mss_uart1, "\r\n  Flash read success \n\r" );
    MSS_UART_polled_tx_string( &g_mss_uart1, g_separator );


}


/**
 *  Read from flash
 */
static void read__flash(uint8_t *read_buf,uint32_t read_byte_size)
{
    uint16_t status;
    int flash_address = 0;

    uint32_t nb_segments_to_read;


    flash_content_t flash_content;
    uint8_t manufacturer_id;
   	 uint8_t device_id;
   	// volatile uint32_t errors = 0;
   	// uint32_t address = 0;
   	 uint32_t count,file_read_byte_length;
   	 //uint8_t * addr_ptr;
   	 uint8_t * write_buf_ptr;

    /*--------------------------------------------------------------------------
         * Flash Driver Initialization
        */
        FLASH_init();

        FLASH_global_unprotect();

        nb_segments_to_read = read_byte_size / FLASH_SEGMENT_SIZE;
           if((read_byte_size % FLASH_SEGMENT_SIZE) > 0)
           {
           	++nb_segments_to_read;
           }


    for( count = 0; count != nb_segments_to_read; ++count )
    {
        /*----------------------------------------------------------------------
         * Write our values to the FLASH, read them back and compare.
         * Placing a breakpoint on the while statement below will allow
         * you break on any failures.
         */

        //spi_flash_read ( flash_address, read_buf, FLASH_SEGMENT_SIZE );
        FLASH_read(flash_address, read_buf, FLASH_SEGMENT_SIZE);
        read_buf += FLASH_SEGMENT_SIZE;

        flash_address += FLASH_SEGMENT_SIZE; /* Step to the next 256 byte chunk */

       // show_progress();
    }

    MSS_UART_polled_tx_string( &g_mss_uart1, "\r\n  Flash read success \n\r" );
    MSS_UART_polled_tx_string( &g_mss_uart1, g_separator );


}









void SysTick_Handler( void )
{
    uint32_t gpio_pattern;
    static uint8_t count;
    /*
     * Toggle GPIO output pattern by doing an exclusive OR of all
     * pattern bits with ones.
     */
    if(count++>=50)
    {
        //gpio_pattern = GPIO_get_outputs( &g_gpio );
    	gpio_pattern  = MSS_GPIO_get_outputs();

        gpio_pattern ^= 0x00000002;
       // GPIO_set_outputs( &g_gpio, gpio_pattern );
        //GPIO_set_outputs( &g_gpio, gpio_pattern );
        MSS_GPIO_set_outputs(gpio_pattern);
        count=0;
    }

    g_10ms_count += 10;

     /*
      * For neatness, if we roll over, reset cleanly back to 0 so the count
      * always goes up in proper 10s.
      */
    if(g_10ms_count < 10)
        g_10ms_count = 0;
}

/*------------------------------------------------------------------------------
 * Call this function if you want to switch to another program
 * de-init any loaded drivers before calling this function
 */
//volatile uint32_t cj_debug;
static void Bootloader_JumpToApplication(uint32_t stack_location, uint32_t reset_vector)
{

	int * address = (int *)0x00000004; 		 //pointer to reset handler of application
	//	copy_spi_image();

		/*********** Boot the Application image from DDR *****************/
	//	delay(100);


	 __set_MSP(*(int*)0xA0000000); 			 //set the stack pointer to that of the application

	    *(volatile uint32_t *)0x40038008 = 0x1;  //Remapping DDR address space to 0x00000000



	   ((void (*)())(*address))(); 			 // pointer recast as function pointer and the dereferenced/called

	   while(1){ }; 						     //This instruction never executed
}






static void Init_UART(void){

	/*--------------------------------------------------------------------------
	     * Initialize and configure UART.
	     */
	    MSS_UART_init(&g_mss_uart1,
	                  MSS_UART_57600_BAUD,
	                  MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

}
static void Init_GPIO(void) {


/*
    * Initialize MSS GPIOs.
    */
   MSS_GPIO_init();

   /*
    * Configure MSS GPIOs.
    */
   	   MSS_GPIO_config( MSS_GPIO_0 , MSS_GPIO_OUTPUT_MODE );
       MSS_GPIO_config( MSS_GPIO_1 , MSS_GPIO_OUTPUT_MODE );
       MSS_GPIO_config( MSS_GPIO_2 , MSS_GPIO_OUTPUT_MODE );
       MSS_GPIO_config( MSS_GPIO_3 , MSS_GPIO_OUTPUT_MODE );
       MSS_GPIO_config( MSS_GPIO_4 , MSS_GPIO_OUTPUT_MODE );
       MSS_GPIO_config( MSS_GPIO_8 , MSS_GPIO_OUTPUT_MODE );
       MSS_GPIO_config( MSS_GPIO_9 , MSS_GPIO_OUTPUT_MODE );
       MSS_GPIO_config( MSS_GPIO_10 , MSS_GPIO_OUTPUT_MODE );


   MSS_GPIO_config( MSS_GPIO_11 , MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
   MSS_GPIO_config( MSS_GPIO_12 , MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_NEGATIVE );


       /*
        * Enable interrupts.
        */

     //  MSS_GPIO_enable_irq(MSS_GPIO_11);
     //  MSS_GPIO_enable_irq(MSS_GPIO_12);

}



static void Init_Timer(void){

	/*--------------------------------------------------------------------------
	     * Configure Timer1
	     */
	    /*
	     * Use the timer input frequency as load value to achieve a one second
	     * periodic interrupt.
	     */
	    timer1_load_value = g_FrequencyPCLK0;
	    MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE);
	    MSS_TIM1_load_immediate(timer1_load_value);
	    //MSS_TIM1_start();
	    //MSS_TIM1_enable_irq();
	    MSS_TIM1_disable_irq();
	    MSS_TIM1_stop();
}








