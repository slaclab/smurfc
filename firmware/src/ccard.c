/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    ccard.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "ccard.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// Taken from ccard.h
uint32_t tes_reset_port[NUM_TES_CHANNELS] = { 4,  6,  6,  4,  0,  5,  3,  3,  3,  3,  3,  3,  2,  0,  3,  0,  4  };
uint32_t tes_reset_bit[NUM_TES_CHANNELS]  = { 3,  13, 14, 0,  6,  0,  6,  4,  12, 11, 3,  9,  14, 15, 0,  4,  5  };
uint32_t tes_set_port[NUM_TES_CHANNELS]   = { 4,  4,  6,  4,  0,  5,  3,  3,  3,  3,  3,  3,  3,  0,  2,  0,  6  };
uint32_t tes_set_bit[NUM_TES_CHANNELS]    = { 4,  2,  12, 1,  7,  1,  7,  5,  13, 10, 2,  8,  1,  14, 13, 5,  15 };

uint32_t RELAY_DEFAULT = 0x00; 
bool RELAY_LATCHING = true; 

typedef uint32_t SPI_DATA_TYPE;  
DRV_HANDLE SPIHandle;
DRV_SPI_BUFFER_HANDLE Write_Buffer_Handle; // Write buffer handle
DRV_SPI_BUFFER_HANDLE Read_Buffer_Handle; // Read buffer handle 
SPI_DATA_TYPE TXbuffer[6]; // SPI Driver TX buffer
SPI_DATA_TYPE RXbuffer[6]; // SPI Driver RX buffer
uint32_t SPI_BYTES = 4; // So far this is all that works. 

DRV_SPI_BUFFER_EVENT test;

uint32_t data_bits = 20; // number of data bits
static inline bool cmd_read(uint32_t data){ return(!!(data & 0x80000000));};  // check read bit
static inline uint32_t cmd_address (uint32_t data) {return((data & 0x7FF00000)>> 20);};
static inline uint32_t cmd_data (uint32_t data){return(data & 0xFFFFF);};  
static inline uint32_t make_cmd(bool read, uint32_t address, uint32_t data )
    {return((read << 31) | ((address & (1 << (32-data_bits-1))-1) << data_bits) | (data & ((1 <<data_bits)-1)) );};

    
uint32_t system_id = 0xABCDE;  // just for testing

// *****************************************************************************
// Taken from ccard.c
uint32_t relay; 
uint32_t response; // return from card for testing
uint32_t cycle_count=0;
uint32_t last_cycle_count = 0;
uint32_t count_increment = 100000;
uint32_t command_count = 0;

uint32_t command; // command from controller
uint32_t addr;
uint32_t default_addr = 0x00; // used until set to some specific value,
uint32_t data;
bool rd;  // read / write data

bool relay_busy = false; 

#define num_test_commands 100
uint32_t CMD[num_test_commands];
uint32_t status = 0; 
uint32_t return_data; 
uint32_t *regptr[ADDR_COUNT];  // will hold pointers to various registers

uint32_t adc_data[16];
uint32_t hemt_bias;
uint32_t a50k_bias;
uint32_t temperature;

uint32_t n;

uint32_t ps_en;     // Power supplies (HEMT and 50k) enable
uint32_t ac_dc_status; // AC/DC mode relay status

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

CCARD_DATA ccardData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void CCARD_Initialize ( void )

  Remarks:
    See prototype in ccard.h.
 */

void CCARD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ccardData.state = CCARD_STATE_INIT;

    
    TXbuffer[0] = 0;
}


/******************************************************************************
  Function:
    void CCARD_Tasks ( void )

  Remarks:
    See prototype in ccard.h.
 */

void CCARD_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( ccardData.state )
    {
        /* Application's initial state. */
        case CCARD_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
                cycle_count = 0;
                relay = RELAY_DEFAULT; // clear all relays
                TES_relay_set(relay); // set relays to default
                relay_busy = true;
                ccardData.hDelayTimer = SYS_TMR_DelayMS(RELAY_DELAY); // start relay timer

                // Disable power supplies
                PS_HEMT_ENOff();
                PS_50k_ENOff();

                // set up register map
                regptr[ADDR_ID] = &system_id; //read back relays by default
                regptr[ADDR_STATUS] = &status;
                regptr[ADDR_RELAY] = &relay; 
                regptr[ADDR_HEMT_BIAS] = &hemt_bias;
                regptr[ADDR_50K_BIAS] = &a50k_bias;
                regptr[ADDR_TEMPERATURE] = &temperature; 
                regptr[ADDR_COUNTER] = &cycle_count;
                regptr[ADDR_PS_EN] = &ps_en;
                regptr[ADDR_AC_DC_STATUS] = &ac_dc_status;
                 
                SPIHandle = DRV_SPI_Open(DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE );  // this is the SPI used for receiving commands
                Read_Buffer_Handle = DRV_SPI_BufferAddWriteRead(SPIHandle,(SPI_DATA_TYPE *)& TXbuffer[0], SPI_BYTES, (SPI_DATA_TYPE *)& RXbuffer[0], SPI_BYTES,0,0);// read buffer
                
                DRV_ADC_Open();
                DRV_ADC_Start(); // start ADC running
                
                ccardData.state = CCARD_STATE_SERVICE_TASKS;
            }
            break;
        }

        case CCARD_STATE_SERVICE_TASKS:
        {
            if (SYS_TMR_DelayStatusGet(ccardData.hDelayTimer)) // relay timer timed out
            {
                ccardData.state = CCARD_RELAY_TIMEOUT; // need to clear relays
                break;
            }
            if (DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(Read_Buffer_Handle)) // check for SPI data
            {
                ccardData.state = CCARD_READ_SPI;  // need to read SPI data
                break;
            }
            if (DRV_ADC_SamplesAvailable())
            {
               ccardData.state = CCARD_READ_ADC;
                break;
            }
            break;
        }
        
        case CCARD_RELAY_TIMEOUT:  // reset relays for latching mode of operation
        {
            if (RELAY_LATCHING)
            {
                TES_relay_clear(); // clears relay drive
            }
            relay_busy = false;  // relay done 
            relay = relay & ((1 << (data_bits-1))-1);  // clear relay busy bit
            ccardData.state = CCARD_STATE_SERVICE_TASKS;
            break;
        }

        /* TODO: implement your application state machine.*/
        case CCARD_READ_SPI:  // this is where we receive commands
        {
            cycle_count++;
            command = RXbuffer[0]; // lock in command for decoding
            rd = cmd_read(command);
            addr = cmd_address (command);
            data = cmd_data (command);  
            ccardData.state = CCARD_STATE_SERVICE_TASKS;   // may be overridden later
            if (rd)  // read command received.
            {                
                if (addr < ADDR_COUNT)  // address in range
                {
                    // If we are reading the AC/DC mode relays status, update the
                    // status work with the readback value
                    if (addr == ADDR_AC_DC_STATUS)
                    {
                        ac_dc_status = (FRP_RLYStateGet() << 1) | FRN_RLYStateGet();
                    }
                    TXbuffer[0] = make_cmd(0,  addr, *regptr[addr] );
                  //  TXbuffer[0] = 0x01;
                    default_addr = addr; // this is now the default read back. 
                } else
                {
                    TXbuffer[0] = make_cmd(0, default_addr, *regptr[default_addr]);
                }
                Read_Buffer_Handle = DRV_SPI_BufferAddWriteRead(SPIHandle, (SPI_DATA_TYPE *)& TXbuffer[0], SPI_BYTES, (SPI_DATA_TYPE *)& RXbuffer[0], SPI_BYTES,0,0); // start new data read
            }
            else // write command
            { 
                TXbuffer[0] = make_cmd(0,  default_addr, *regptr[default_addr] );
                //TXbuffer[0] = 0x01;
                switch (addr)
                {
                    case ADDR_RELAY:
                    {
                        relay = data | (1 << (data_bits -1)); // set bit to show that relays are in motion
                        if (relay_busy) break;  // just ignore if relay already busy 
                        relay_busy = true;
                        TES_relay_set(relay); // set relays to default
                        ccardData.hDelayTimer = SYS_TMR_DelayMS(RELAY_DELAY); // start relay timer 
                        break;
                    }
                    case ADDR_PS_EN:
                    {
                        ps_en = data & 0x03;  // Only 2 bits are used
                        PS_HEMT_ENStateSet( ps_en & 0x01 );        // HEMT_EN
                        PS_50k_ENStateSet( ( ps_en >> 1 ) & 0x01 );  // 50k_EN
                        break;
                    }
                    default:
                    {
                        break;
                    }    
                }
                Read_Buffer_Handle = DRV_SPI_BufferAddWriteRead(SPIHandle, (SPI_DATA_TYPE *)& TXbuffer[0], SPI_BYTES, (SPI_DATA_TYPE *)& RXbuffer[0], SPI_BYTES,0,0); // start new data read
            }
            break;
        }
        
        case CCARD_READ_ADC:
        {
            DRV_ADC_Stop();
            for (n = 0; n < 16; n++) { adc_data[n] = DRV_ADC_SamplesRead(n);}  // KLUDGE< first points seem bad!
            DRV_ADC_Start(); // start ADC running again
            hemt_bias = 0;
            a50k_bias = 0;
            temperature = 0; // average 5 samples each
            for (n = ADC_HEMT_BIAS_CHAN; n < 15; n = n + 3) {hemt_bias += adc_data[n];}
            for (n = ADC_50K_BIAS_CHAN; n < 15; n = n + 3) {a50k_bias += adc_data[n];}
            for (n = ADC_TEMPERATURE_CHAN; n < 15; n = n + 3) {temperature += adc_data[n];}        
            ccardData.state = CCARD_STATE_SERVICE_TASKS;
            break;
         }   

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
